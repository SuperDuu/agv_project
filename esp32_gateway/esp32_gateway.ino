#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <FirebaseESP32.h> // Cài đặt thư viện "Firebase ESP32 Client" của Mobizt
#include <Wire.h>
#include <string.h>
#include "src/bno055.h"
#include "src/SparkFun_VL53L5CX_Library.h"

// ==========================================
// CẤU HÌNH PHẦN CỨNG & MẶC ĐỊNH
// ==========================================
#define BOOT_BUTTON_PIN 0 // Chân nút BOOT trên ESP32 (Thường là GPIO 0)
#define AP_SSID "Du"
#define AP_PASS "du123456"

// Cấu hình chân I2C cho các cảm biến (BNO055, VL53L5CX)
#define I2C_SDA 21
#define I2C_SCL 22

// Sử dụng Hardware Serial 2 của ESP32 để giao tiếp STM32
#define RXD2 33
#define TXD2 26
#define UART_BAUDRATE 115200
#define HC12_RXD 16
#define HC12_TXD 17
#define HC12_BAUDRATE 115200
#define ARM_CMD_MAX_LEN 48
#define PROTO_SOF1 0xAA
#define PROTO_SOF2 0x55
#define PROTO_ADDR_MAIN 0x01
#define PROTO_ADDR_ARM_LEFT 0x02
#define PROTO_ADDR_ARM_RIGHT 0x03
#define PROTO_ADDR_ESP32 0x10
#define PROTO_ADDR_PC_APP 0x20
#define PROTO_CMD_SENSOR_REPORT 0x01
#define PROTO_CMD_SYNC_REQUEST 0x11
#define PROTO_CMD_ARM_JOINT_COMMAND 0x20
#define PROTO_MOTION_ABSOLUTE 0x01
#define PROTO_SENSOR_FLAG_IMU_VALID 0x01
#define PROTO_SENSOR_FLAG_VL53_VALID 0x02
#define PROTO_SENSOR_FLAG_NEW_TARGET 0x04
#define PROTO_MAX_PAYLOAD_LEN 64
#define PROTO_MAX_FRAME_LEN 74

#define RS485_ADDR 0x63  // Địa chỉ 99 cho Master-Slave IMU

// ==========================================
// KHAI BÁO BIẾN TOÀN CỤC
// ==========================================
Preferences preferences; // Thư viện NVS thay thế EEPROM
WebServer server(80);    // Khởi tạo WebServer ở cổng 80

TaskHandle_t FirebaseTaskHandle; // Task đa luồng cho Firebase

// Biến lưu trữ cấu hình mạng (Sẽ được nạp từ bộ nhớ)
String wifi_ssid = "";
String wifi_pass = "";
String fb_host = "";
String fb_auth = "";

// Biến Firebase
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

String currentCommand = ""; 
double currentTimestamp = 0;
bool isAckReceived = true;  
unsigned long lastSendTime = 0; 
unsigned long lastCheckTime = 0;
const int FIREBASE_CHECK_INTERVAL = 2000; 
bool isConfigMode = false; // Cờ kiểm tra đang ở chế độ Web hay chế độ Chạy

// Biến cho giao thức Master-Slave IMU (RS485)
uint16_t current_node = 0xFFFF;
float current_yaw = 0.0f; // Góc mô phỏng BNO055
float imu_total_yaw = 0.0f;  // Góc tích lũy không bị nhảy qua -180/180
float imu_prev_yaw = 0.0f;
float imu_delta_yaw = 0.0f;
uint8_t main_rx_frame[PROTO_MAX_FRAME_LEN];
uint16_t main_rx_index = 0;
uint16_t main_expected_len = 0;
uint8_t main_rx_state = 0;

uint8_t hc12_rx_frame[PROTO_MAX_FRAME_LEN];
uint16_t hc12_rx_index = 0;
uint16_t hc12_expected_len = 0;
uint8_t hc12_rx_state = 0;
uint8_t esp32_seq_counter = 0;
uint8_t rs485_rx_buffer[10];
uint8_t rs485_rx_index = 0;
unsigned long last_rs485_rx_time = 0;

uint8_t firebase_node = 255;  // 255 = Không có lệnh
uint8_t firebase_h_cmd = 255; // 255 = Không có lệnh
unsigned long last_firebase_cmd_time = 0;

// Biến trạng thái gửi lên Firebase
uint8_t current_arrived_status = 255;
bool need_send_status = false;

// Biến quản lý trạng thái cảm biến (Fail-safe)
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;

bool bno055_ok = false;
bool vl53_ok = false;
uint16_t obstacle_distance = 0xFFFF; // 0xFFFF = Báo lỗi xuống STM32 nếu đứt dây

void updateImuAccumulatedYaw() {
  if (!bno055_ok) return;

  bno055_vector_t v = bno055_getVectorEuler();
  float cur0 = v.x;
  float delta0 = cur0 - imu_prev_yaw;

  if (delta0 > 180.0f) delta0 -= 360.0f;
  if (delta0 < -180.0f) delta0 += 360.0f;

  imu_total_yaw += delta0;
  imu_prev_yaw = cur0;
  current_yaw = cur0;
  imu_delta_yaw = delta0;
}

unsigned long lastSensorDebugTime = 0;
const unsigned long SENSOR_DEBUG_INTERVAL_MS = 100;
unsigned long lastRs485TestTxTime = 0;
const unsigned long RS485_TEST_TX_INTERVAL_MS = 200;
HardwareSerial HC12Serial(1);
char arm_cmd_buffer[ARM_CMD_MAX_LEN + 1];
uint8_t arm_cmd_index = 0;

// Hàm tính XOR Checksum cho khung nhị phân
uint16_t proto_crc16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;

  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

/* --- Protocol V2.1: Little-Endian helpers -------------------------------- */
void write_u16_le(uint8_t *dst, uint16_t value) {
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)((value >> 8) & 0xFF);
}

void write_s16_le(uint8_t *dst, int16_t value) {
  write_u16_le(dst, (uint16_t)value);
}

uint16_t read_u16_le(const uint8_t *src) {
  return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

int16_t read_s16_le(const uint8_t *src) {
  return (int16_t)read_u16_le(src);
}

/* Compatibility aliases — remove once all call sites are updated */
#define write_u16_be write_u16_le
#define write_s16_be write_s16_le
#define read_u16_be  read_u16_le

int16_t deg_to_x100(float angle_deg) {
  if (angle_deg >= 0.0f) return (int16_t)(angle_deg * 100.0f + 0.5f);
  return (int16_t)(angle_deg * 100.0f - 0.5f);
}

uint8_t calculate_checksum(uint8_t *data, uint8_t start, uint8_t end) {
  uint8_t cs = 0;
  for (uint8_t i = start; i <= end; i++) cs ^= data[i];
  return cs;
}

// Xử lý gói tin nhị phân nhận từ STM32
bool parse_arm_command_line(const char *cmd, uint8_t *dest, int16_t joints_x100[6]) {
  if (cmd == nullptr) return false;

  size_t len = strlen(cmd);
  if (len < 4 || len > ARM_CMD_MAX_LEN || cmd[1] != ':') return false;

  if (cmd[0] == 'L') {
    *dest = PROTO_ADDR_ARM_LEFT;
  } else if (cmd[0] == 'R') {
    *dest = PROTO_ADDR_ARM_RIGHT;
  } else {
    return false;
  }

  char local_buf[ARM_CMD_MAX_LEN + 1];
  strncpy(local_buf, cmd + 2, sizeof(local_buf) - 1);
  local_buf[sizeof(local_buf) - 1] = '\0';

  char *context = nullptr;
  char *token = strtok_r(local_buf, ",", &context);
  for (int i = 0; i < 6; i++) {
    if (token == nullptr) return false;
    joints_x100[i] = deg_to_x100(strtof(token, nullptr));
    token = strtok_r(nullptr, ",", &context);
  }

  return token == nullptr;
}

/* --- V2.1 frame layout ---------------------------------------------------
 * [SOF1][SOF2][DEST][SRC][LEN_L][LEN_H][CMD][SEQ][PAYLOAD...][CRC_L][CRC_H]
 *   0     1     2    3     4      5      6    7     8..        -2    -1
 * LEN = payload_len (LE), CRC covers bytes [2 .. 8+payload_len-1]
 * ----------------------------------------------------------------------- */
void send_proto_frame(uint8_t dest, uint8_t cmd, const uint8_t *payload, uint16_t payload_len) {
  uint8_t tx_frame[PROTO_MAX_FRAME_LEN];
  if (payload_len > PROTO_MAX_PAYLOAD_LEN) return;

  tx_frame[0] = PROTO_SOF1;
  tx_frame[1] = PROTO_SOF2;
  tx_frame[2] = dest;
  tx_frame[3] = PROTO_ADDR_ESP32;
  write_u16_le(&tx_frame[4], payload_len);   /* LEN at [4:5] — LE */
  tx_frame[6] = cmd;                         /* CMD at [6]        */
  tx_frame[7] = esp32_seq_counter++;         /* SEQ at [7]        */
  if (payload_len > 0 && payload != nullptr) {
    memcpy(&tx_frame[8], payload, payload_len);
  }

  /* CRC covers: DEST, SRC, LEN_L, LEN_H, CMD, SEQ, PAYLOAD = 6 + payload_len bytes */
  uint16_t crc = proto_crc16(&tx_frame[2], (uint16_t)(6 + payload_len));
  write_u16_le(&tx_frame[8 + payload_len], crc);   /* CRC: LE */

  Serial2.write(tx_frame, payload_len + 10);
  Serial2.flush();
}

/* V2.1 joint command: arm_id REMOVED, payload 22 bytes, max_delta_x100 added.
 * DEST field identifies arm (0x02=left, 0x03=right) — no arm_id in payload.
 * max_delta_x100 = 300 → 3.00°/frame safety limit at 50Hz. */
void send_arm_command_frame(const char *cmd) {
  uint8_t dest = 0;
  int16_t joints_x100[6];
  uint8_t payload[22];   /* 22 bytes: no arm_id, + max_delta_x100 */

  if (!parse_arm_command_line(cmd, &dest, joints_x100)) {
    Serial.printf("[HC-12] Loi phan tich lenh: '%s'\n", cmd);
    return;
  }

  Serial.printf("[HC-12] Nhap lenh Arm: %s -> DEST: 0x%02X, Q: q1=%.2f, q2=%.2f, q3=%.2f, q4=%.2f, q5=%.2f, q6=%.2f\n",
                cmd, dest, 
                (float)joints_x100[0] / 100.0f, (float)joints_x100[1] / 100.0f, (float)joints_x100[2] / 100.0f,
                (float)joints_x100[3] / 100.0f, (float)joints_x100[4] / 100.0f, (float)joints_x100[5] / 100.0f);

  /* [0] motion_mode  [1] arm_flags */
  payload[0] = PROTO_MOTION_ABSOLUTE;
  payload[1] = 0x01;                         /* arm_flags: payload valid */
  /* [2..13] joint angles, Little-Endian */
  write_s16_le(&payload[2],  joints_x100[0]);
  write_s16_le(&payload[4],  joints_x100[1]);
  write_s16_le(&payload[6],  joints_x100[2]);
  write_s16_le(&payload[8],  joints_x100[3]);
  write_s16_le(&payload[10], joints_x100[4]);
  write_s16_le(&payload[12], joints_x100[5]);
  /* [14..15] move_time_ms = 0 (use arm default) */
  write_u16_le(&payload[14], 0);
  /* [16..17] max_delta_x100 = 300 (3.00° max per frame at 50Hz) */
  write_u16_le(&payload[16], 300);
  /* [18..21] reserved */
  payload[18] = 0; payload[19] = 0; payload[20] = 0; payload[21] = 0;

  /* DEST encodes which arm: 0x02=left, 0x03=right (from parse_arm_command_line) */
  send_proto_frame(dest, PROTO_CMD_ARM_JOINT_COMMAND, payload, sizeof(payload));
}

void process_hc12_frame(const uint8_t *frame, uint16_t frame_len) {
  if (frame_len < 10 || frame[0] != PROTO_SOF1 || frame[1] != PROTO_SOF2) return;

  uint16_t payload_len = read_u16_le(&frame[4]);
  if ((uint16_t)(payload_len + 10) != frame_len) return;

  uint16_t rx_crc = read_u16_le(&frame[8 + payload_len]);
  uint16_t calc_crc = proto_crc16(&frame[2], (uint16_t)(6 + payload_len));
  if (rx_crc != calc_crc) {
    Serial.printf("[HC-12 RX ERROR] CRC mismatch: rx=0x%04X, calc=0x%04X\n", rx_crc, calc_crc);
    return;
  }

  uint8_t dest = frame[2];
  uint8_t src = frame[3];
  uint8_t cmd = frame[6];
  uint8_t seq = frame[7];

  if (cmd == PROTO_CMD_ARM_JOINT_COMMAND && payload_len == 22) {
    int16_t q1 = read_s16_le(&frame[8 + 2]);
    int16_t q2 = read_s16_le(&frame[8 + 4]);
    int16_t q3 = read_s16_le(&frame[8 + 6]);
    int16_t q4 = read_s16_le(&frame[8 + 8]);
    int16_t q5 = read_s16_le(&frame[8 + 10]);
    int16_t q6 = read_s16_le(&frame[8 + 12]);
    uint16_t max_delta = read_u16_le(&frame[8 + 16]);

    Serial.printf("[HC-12 RX OK] Arm Joint: DEST=0x%02X, SRC=0x%02X, SEQ=%u, Q: q1=%.2f, q2=%.2f, q3=%.2f, q4=%.2f, q5=%.2f, q6=%.2f, max_delta=%.2f\n",
                  dest, src, seq,
                  (float)q1 / 100.0f, (float)q2 / 100.0f, (float)q3 / 100.0f,
                  (float)q4 / 100.0f, (float)q5 / 100.0f, (float)q6 / 100.0f,
                  (float)max_delta / 100.0f);
  } else {
    Serial.printf("[HC-12 RX OK] Frame: DEST=0x%02X, SRC=0x%02X, CMD=0x%02X, SEQ=%u, LEN=%u\n",
                  dest, src, cmd, seq, payload_len);
  }

  // Forward the valid binary frame to STM32 (Serial2)
  Serial2.write(frame, frame_len);
  Serial2.flush();
}

void process_hc12_uart() {
  while (HC12Serial.available() > 0) {
    uint8_t b = (uint8_t)HC12Serial.read();

    switch (hc12_rx_state) {
      case 0:
        if (b == PROTO_SOF1) {
          hc12_rx_frame[0] = b;
          hc12_rx_index = 1;
          hc12_rx_state = 1;
        }
        break;

      case 1:
        if (b == PROTO_SOF2) {
          hc12_rx_frame[1] = b;
          hc12_rx_index = 2;
          hc12_rx_state = 2;
        } else if (b == PROTO_SOF1) {
          hc12_rx_frame[0] = b;
          hc12_rx_index = 1;
        } else {
          hc12_rx_state = 0;
          hc12_rx_index = 0;
        }
        break;

      case 2:
        if (hc12_rx_index >= PROTO_MAX_FRAME_LEN) {
          hc12_rx_state = 0;
          hc12_rx_index = 0;
          hc12_expected_len = 0;
          break;
        }

        hc12_rx_frame[hc12_rx_index++] = b;

        /* LEN at offset [4:5] (LE) — known after receiving 6 bytes (index==6) */
        if (hc12_rx_index == 6) {
          uint16_t payload_len = read_u16_le(&hc12_rx_frame[4]);
          if (payload_len > PROTO_MAX_PAYLOAD_LEN) {
            hc12_rx_state = 0;
            hc12_rx_index = 0;
            hc12_expected_len = 0;
            break;
          }
          hc12_expected_len = (uint16_t)(payload_len + 10);
        }

        if (hc12_expected_len != 0 && hc12_rx_index == hc12_expected_len) {
          process_hc12_frame(hc12_rx_frame, hc12_expected_len);
          hc12_rx_state = 0;
          hc12_rx_index = 0;
          hc12_expected_len = 0;
        }
        break;

      default:
        hc12_rx_state = 0;
        hc12_rx_index = 0;
        hc12_expected_len = 0;
        break;
    }
  }
}

void parse_rs485_frame() {
  if (rs485_rx_index == 8) {
    // --- DEBUG RX ---
    // Serial.print("[RX]: ");
    // for(int i=0; i<8; i++) {
    //   Serial.printf("%02X ", rs485_rx_buffer[i]);
    // }
    // ----------------

    if (rs485_rx_buffer[0] == 0xAA && rs485_rx_buffer[1] == 0x55) {
      if (rs485_rx_buffer[2] == RS485_ADDR) {
        if (calculate_checksum(rs485_rx_buffer, 2, 6) == rs485_rx_buffer[7]) {
          // STM32 yêu cầu lấy Yaw và gửi Node hiện tại
          if (rs485_rx_buffer[3] == 0x01) {
            current_node = (rs485_rx_buffer[4] << 8) | rs485_rx_buffer[5];
            
            // Đọc trạng thái đã đến đích từ STM32
            uint8_t is_arrived = rs485_rx_buffer[6];
            if (is_arrived != current_arrived_status && is_arrived <= 1) {
                current_arrived_status = is_arrived;
                need_send_status = true;
            }

            // Đọc thực tế từ BNO055
            int16_t yaw_int = 0x7FFF; // Mặc định là lỗi đứt dây (0x7FFF thay vì 0xFFFF để tránh trùng góc âm)
            if (bno055_ok) {
              updateImuAccumulatedYaw();
              yaw_int = (int16_t)(imu_total_yaw * 10.0f);
              if (yaw_int == 0x7FFF) yaw_int = 0x7FFE; // Tránh trùng mã lỗi
            }

            uint8_t tx_frame[11];
            tx_frame[0] = 0xAA; tx_frame[1] = 0x55;
            tx_frame[2] = RS485_ADDR; tx_frame[3] = 0x01;
            tx_frame[4] = (yaw_int >> 8) & 0xFF;
            tx_frame[5] = yaw_int & 0xFF;
            
            // Gửi kèm lệnh Firebase mới nhất
            tx_frame[6] = firebase_node;
            tx_frame[7] = firebase_h_cmd;
            
            // Gửi kèm khoảng cách siêu âm
            tx_frame[8] = (obstacle_distance >> 8) & 0xFF;
            tx_frame[9] = obstacle_distance & 0xFF;
            
            tx_frame[10] = calculate_checksum(tx_frame, 2, 9);
            Serial.printf("%d\n", obstacle_distance);
            // --- DEBUG TX ---
            /*
            Serial.printf(" => [TX] yaw=%.1fdeg node=%u cmd=%u obs=%u | ",
                          imu_total_yaw,
                          (unsigned)firebase_node,
                          (unsigned)firebase_h_cmd,
                          (unsigned)obstacle_distance);
            for (int i = 0; i < 11; i++) {
              Serial.printf("%02X ", tx_frame[i]);
            }
            Serial.println();
            */
            // ----------------

            // Gửi ngay sau khi tạo frame để giảm độ trễ và tránh block loop
            // (Nếu phần RS485 cần turnaround riêng thì xử lý bằng phần cứng/DE pin,
            // không nên dùng delay chặn trong vòng lặp chính)
            Serial2.write(tx_frame, 11);
            Serial2.flush();
            
            // Xóa cờ sau khi đã gửi 1 giây để đảm bảo STM32 nhận được (tránh lỗi rớt gói RS485)
            if (firebase_node != 255 && last_firebase_cmd_time == 0) {
                last_firebase_cmd_time = millis();
            }
            if (firebase_node != 255 && millis() - last_firebase_cmd_time > 1000) {
                firebase_node = 255;
                firebase_h_cmd = 255;
                last_firebase_cmd_time = 0;
            }
          } else { Serial.println(" => (Loi CMD)"); }
        } else { Serial.println(" => (Loi Checksum)"); }
      } else { Serial.println(" => (Loi Dia Chi)"); }
    } else { Serial.println(" => (Loi Header)"); }
  }
  rs485_rx_index = 0;
}

void send_sensor_report_frame() {
  uint8_t payload[8];
  int16_t yaw_x100 = 0;
  uint8_t sensor_flags = 0;

  if (bno055_ok) {
    yaw_x100 = deg_to_x100(imu_total_yaw);
    sensor_flags |= PROTO_SENSOR_FLAG_IMU_VALID;
  }

  if (vl53_ok) {
    sensor_flags |= PROTO_SENSOR_FLAG_VL53_VALID;
  }

  if (firebase_node != 255) {
    sensor_flags |= PROTO_SENSOR_FLAG_NEW_TARGET;
  }

  write_s16_le(&payload[0], yaw_x100);
  write_u16_le(&payload[2], obstacle_distance);
  write_u16_le(&payload[4], firebase_node == 255 ? 0xFFFF : (uint16_t)firebase_node);
  payload[6] = firebase_h_cmd;
  payload[7] = sensor_flags;

  send_proto_frame(PROTO_ADDR_MAIN, PROTO_CMD_SENSOR_REPORT, payload, sizeof(payload));

  if (firebase_node != 255 && last_firebase_cmd_time == 0) {
    last_firebase_cmd_time = millis();
  }
  if (firebase_node != 255 && millis() - last_firebase_cmd_time > 1000) {
    firebase_node = 255;
    firebase_h_cmd = 255;
    last_firebase_cmd_time = 0;
  }
}

/* V2.1 frame layout received from AGV main:
 * [SOF1][SOF2][DEST][SRC][LEN_L][LEN_H][CMD][SEQ][PAYLOAD...][CRC_L][CRC_H]
 *   0     1     2    3     4      5      6    7     8..        -2    -1      */
void process_main_frame(const uint8_t *frame, uint16_t frame_len) {
  if (frame_len < 10 || frame[0] != PROTO_SOF1 || frame[1] != PROTO_SOF2) return;
  if (frame[2] != PROTO_ADDR_ESP32) return;

  uint16_t payload_len = read_u16_le(&frame[4]);   /* LEN at [4:5] LE */
  if ((uint16_t)(payload_len + 10) != frame_len) return;

  uint16_t rx_crc   = read_u16_le(&frame[8 + payload_len]);
  uint16_t calc_crc = proto_crc16(&frame[2], (uint16_t)(6 + payload_len));
  if (rx_crc != calc_crc) return;

  uint8_t cmd = frame[6];   /* CMD at [6], SEQ at [7] */

  if (cmd == PROTO_CMD_SYNC_REQUEST && payload_len == 4) {
    current_node = read_u16_le(&frame[8]);
    uint8_t is_arrived = frame[10];
    Serial.printf("[STM32 Sync] Nhan yeu cau: Node hien tai = %u, Da den = %d\n", current_node, is_arrived);
    if (is_arrived != current_arrived_status && is_arrived <= 1) {
      current_arrived_status = is_arrived;
      need_send_status = true;
    }
    send_sensor_report_frame();
  }
}

void process_main_uart() {
  while (Serial2.available() > 0) {
    uint8_t b = (uint8_t)Serial2.read();

    switch (main_rx_state) {
      case 0:
        if (b == PROTO_SOF1) {
          main_rx_frame[0] = b;
          main_rx_index = 1;
          main_rx_state = 1;
        }
        break;

      case 1:
        if (b == PROTO_SOF2) {
          main_rx_frame[1] = b;
          main_rx_index = 2;
          main_rx_state = 2;
        } else if (b == PROTO_SOF1) {
          main_rx_frame[0] = b;
          main_rx_index = 1;
        } else {
          main_rx_state = 0;
          main_rx_index = 0;
        }
        break;

      case 2:
        if (main_rx_index >= PROTO_MAX_FRAME_LEN) {
          main_rx_state = 0;
          main_rx_index = 0;
          main_expected_len = 0;
          break;
        }

        main_rx_frame[main_rx_index++] = b;

        /* LEN at offset [4:5] (LE) — known after receiving 6 bytes (index==6) */
        if (main_rx_index == 6) {
          uint16_t payload_len = read_u16_le(&main_rx_frame[4]);
          if (payload_len > PROTO_MAX_PAYLOAD_LEN) {
            main_rx_state = 0;
            main_rx_index = 0;
            main_expected_len = 0;
            break;
          }
          main_expected_len = (uint16_t)(payload_len + 10);
        }

        if (main_expected_len != 0 && main_rx_index == main_expected_len) {
          process_main_frame(main_rx_frame, main_expected_len);
          main_rx_state = 0;
          main_rx_index = 0;
          main_expected_len = 0;
        }
        break;

      default:
        main_rx_state = 0;
        main_rx_index = 0;
        main_expected_len = 0;
        break;
    }
  }
}

// ==========================================
// GIAO DIỆN WEB (HTML + CSS CƠ BẢN)
// ==========================================
// Dùng Raw String Literal (R"=====(...)=====") để viết HTML ngay trong code C++ mà không cần escape ký tự
const char* HTML_CONTENT = R"=====(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>AGV Gateway Setup</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;700&display=swap');
  body { font-family: 'Inter', sans-serif; background: linear-gradient(135deg, #1e293b, #0f172a); color: #f8fafc; margin: 0; padding: 20px; min-height: 100vh; display: flex; align-items: center; justify-content: center; }
  .card { background: rgba(255, 255, 255, 0.05); backdrop-filter: blur(12px); -webkit-backdrop-filter: blur(12px); padding: 35px 30px; border-radius: 20px; box-shadow: 0 20px 40px rgba(0,0,0,0.4); width: 100%; max-width: 400px; border: 1px solid rgba(255, 255, 255, 0.1); }
  .header { text-align: center; margin-bottom: 30px; }
  .header h2 { margin: 0; color: #38bdf8; font-weight: 700; letter-spacing: 1.5px; font-size: 24px; text-shadow: 0 2px 4px rgba(0,0,0,0.3); }
  .header p { color: #94a3b8; font-size: 14px; margin-top: 8px; }
  label { font-weight: 600; font-size: 13px; color: #cbd5e1; display: block; margin-top: 18px; margin-bottom: 6px; }
  input, select { width: 100%; padding: 12px 14px; background: rgba(0, 0, 0, 0.25); border: 1px solid #334155; border-radius: 10px; color: #fff; font-size: 15px; transition: all 0.3s ease; box-sizing: border-box; }
  input:focus, select:focus { outline: none; border-color: #38bdf8; box-shadow: 0 0 0 3px rgba(56, 189, 248, 0.2); background: rgba(0, 0, 0, 0.4); }
  button { width: 100%; padding: 14px; margin-top: 30px; background: linear-gradient(135deg, #0ea5e9, #0284c7); color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: 700; cursor: pointer; transition: all 0.3s ease; text-transform: uppercase; letter-spacing: 1px; box-shadow: 0 4px 12px rgba(14, 165, 233, 0.3); }
  button:hover { background: linear-gradient(135deg, #38bdf8, #0ea5e9); transform: translateY(-2px); box-shadow: 0 6px 15px rgba(14, 165, 233, 0.4); }
  button:active { transform: translateY(0); box-shadow: 0 2px 8px rgba(14, 165, 233, 0.3); }
  .footer { text-align: center; margin-top: 25px; font-size: 12px; color: #64748b; }
</style>
</head><body>
<div class="card">
  <div class="header">
    <h2>AGV GATEWAY</h2>
    <p>Smart Configuration Panel</p>
  </div>
  <form action="/save" method="POST">
    <label>Tên WiFi 2.4GHz (Bấm để chọn):</label>
    <select name="ssid">%SSID_OPTIONS%</select>
    <div style="margin-top:8px;font-size:12px;color:#fbbf24;line-height:1.5;">
      Lưu ý: ESP32 chỉ kết nối được WiFi 2.4GHz. Nếu router đang phát 5GHz hoặc SSID chỉ có 5GHz, thiết bị sẽ không tìm thấy / không kết nối được.
    </div>
    
    <label>Mật khẩu WiFi:</label>
    <input type="password" name="pass" value="%WIFI_PASS%" placeholder="Để trống nếu không có pass">
    
    <label>Firebase Host (Bỏ https:// và /):</label>
    <input type="text" name="fb_host" value="%FB_HOST%" required placeholder="VD: agv-test.firebaseio.com">
    
    <label>Firebase API Key:</label>
    <input type="text" name="fb_auth" value="%FB_AUTH%" required placeholder="Nhập API Key">
    
    <button type="submit">Lưu & Khởi Động Lại</button>
  </form>
  <div class="footer">Deepmind AGV Systems &copy; 2026</div>
</div>
</body></html>
)=====";

// ==========================================
// CÁC HÀM XỬ LÝ LƯU TRỮ (NVS PREFERENCES)
// ==========================================
void loadConfig() {
  preferences.begin("agv_config", true); // Tham số true = Read-only mode
  wifi_ssid = preferences.getString("ssid", "");
  wifi_pass = preferences.getString("pass", "");
  fb_host = preferences.getString("fb_host", "");
  fb_auth = preferences.getString("fb_auth", "");
  preferences.end();

  Serial.println("[CFG] Da nap cau hinh tu NVS:");
  Serial.printf("[CFG] SSID len = %u, PASS len = %u, FB_HOST len = %u, FB_AUTH len = %u\n",
                wifi_ssid.length(), wifi_pass.length(), fb_host.length(), fb_auth.length());
  if (wifi_ssid.length() > 0) {
    Serial.printf("[CFG] SSID = '%s'\n", wifi_ssid.c_str());
  }
}

void saveConfig(String s, String p, String h, String a) {
  Serial.println("[CFG] Dang luu cau hinh moi vao NVS:");
  Serial.printf("[CFG] SSID len = %u\n", s.length());
  Serial.printf("[CFG] PASS len = %u\n", p.length());
  Serial.printf("[CFG] FB_HOST len = %u\n", h.length());
  Serial.printf("[CFG] FB_AUTH len = %u\n", a.length());
  Serial.printf("[CFG] SSID = '%s'\n", s.c_str());

  preferences.begin("agv_config", false); // Tham số false = Read/Write mode
  preferences.putString("ssid", s);
  preferences.putString("pass", p);
  preferences.putString("fb_host", h);
  preferences.putString("fb_auth", a);
  preferences.end();
}

// ==========================================
// CÁC HÀM HỖ TRỢ KẾT NỐI WIFI
// ==========================================
int findBest2GhzNetwork(const String &targetSsid, uint8_t *outBssid, int *outChannel) {
  const int MAX_SCAN_TRIES = 10;
  const int SCAN_DELAY_MS = 700;

  for (int attempt = 1; attempt <= MAX_SCAN_TRIES; ++attempt) {
    Serial.printf("[WIFI] Bat dau scan mang (lan %d/%d)...\n", attempt, MAX_SCAN_TRIES);
    int n = WiFi.scanNetworks(false, true);
    Serial.printf("[WIFI] So AP tim thay = %d\n", n);

    int bestIndex = -1;
    int bestRssi = -1000;

    for (int i = 0; i < n; ++i) {
      String foundSsid = WiFi.SSID(i);
      int channel = WiFi.channel(i);
      int rssi = WiFi.RSSI(i);
      bool is2G = (channel > 0 && channel <= 14);

      Serial.printf("[WIFI][SCAN] idx=%d ssid='%s' ch=%d rssi=%d dBm band=%s\n",
                    i, foundSsid.c_str(), channel, rssi, is2G ? "2.4G" : "5G/unknown");

      if (foundSsid == targetSsid && is2G) {
        if (rssi > bestRssi) {
          bestRssi = rssi;
          bestIndex = i;
        }
      }
    }

    if (bestIndex >= 0) {
      *outChannel = WiFi.channel(bestIndex);
      memcpy(outBssid, WiFi.BSSID(bestIndex), 6);
      Serial.printf("[WIFI] Chon AP tot nhat idx=%d, ch=%d, rssi=%d dBm\n",
                    bestIndex, *outChannel, WiFi.RSSI(bestIndex));
      WiFi.scanDelete();
      return bestIndex;
    }

    WiFi.scanDelete();

    if (attempt < MAX_SCAN_TRIES) {
      Serial.printf("[WIFI] Chua thay SSID '%s', doi %d ms roi quet lai...\n", targetSsid.c_str(), SCAN_DELAY_MS);
      delay(SCAN_DELAY_MS);
    }
  }

  Serial.println("[WIFI] Quet het 10 lan van khong thay AP 2.4GHz trung khop SSID duoc tim thay.");
  return -1;
}

const char* wifiStatusToText(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
    default: return "UNKNOWN";
  }
}

bool connectWifi24G(const String &ssid, const String &pass) {
  Serial.printf("[WIFI] Dang tim mang 2.4GHz: '%s'\n", ssid.c_str());
  Serial.printf("[WIFI] PASS len = %u\n", pass.length());

  uint8_t bssid[6] = {0};
  int channel = 0;
  int idx = findBest2GhzNetwork(ssid, bssid, &channel);

  if (idx < 0) {
    Serial.println("[WIFI] Khong tim thay mang 2.4GHz phu hop. Neu router co 2 band, hay tach SSID 2.4G rieng.");
    return false;
  }

  Serial.printf("[WIFI] Ket noi vao channel %d, RSSI = %d dBm\n", channel, WiFi.RSSI(idx));
  Serial.printf("[WIFI] BSSID = %02X:%02X:%02X:%02X:%02X:%02X\n",
                bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);

  WiFi.begin(ssid.c_str(), pass.c_str(), channel, bssid, true);

  unsigned long startAttemptTime = millis();
  wl_status_t lastStatus = (wl_status_t)255;
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
    wl_status_t st = WiFi.status();
    if (st != lastStatus) {
      Serial.printf("[WIFI] status = %d (%s)\n", st, wifiStatusToText(st));
      lastStatus = st;
    }
    delay(500);
  }
  Serial.println();

  wl_status_t finalStatus = WiFi.status();
  Serial.printf("[WIFI] Final status = %d (%s)\n", finalStatus, wifiStatusToText(finalStatus));
  Serial.printf("[WIFI] LocalIP = %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("[WIFI] Gateway = %s, Mask = %s, DNS = %s\n",
                WiFi.gatewayIP().toString().c_str(),
                WiFi.subnetMask().toString().c_str(),
                WiFi.dnsIP().toString().c_str());

  if (finalStatus == WL_CONNECTED) {
    Serial.println("[WIFI] Da ket noi thanh cong.");
    return true;
  }

  if (finalStatus == WL_NO_SSID_AVAIL) {
    Serial.println("[WIFI] Loi: khong thay SSID. Hay kiem tra ten mang 2.4GHz co dung 100% khong.");
  } else if (finalStatus == WL_CONNECT_FAILED) {
    Serial.println("[WIFI] Loi: ket noi that bai. Thuong la sai password, WPA khong tuong thich, hoac router chan handshake.");
  } else if (finalStatus == WL_DISCONNECTED) {
    Serial.println("[WIFI] Loi: bi ngat ket noi trong qua trinh bat tay.");
  } else {
    Serial.println("[WIFI] Loi khac: can kiem tra router, DHCP, band steering, MAC filter.");
  }

  return false;
}

// ==========================================
// CÁC HÀM XỬ LÝ WEBSERVER
// ==========================================
void handleRoot() {
  String html = HTML_CONTENT;
  
  // Quét danh sách WiFi xung quanh
  int n = WiFi.scanNetworks();
  if (n == 0) {
    // Nếu quét lần 1 không thấy, thử đợi một chút và quét lại
    delay(500);
    n = WiFi.scanNetworks();
  }
  
  String options = "";
  if (n == 0) {
    options = "<option value=''>Không tìm thấy WiFi nào (hãy thử refresh lại)!</option>";
  } else {
    for (int i = 0; i < n; ++i) {
      String ssidName = WiFi.SSID(i);
      options += "<option value='" + ssidName + "'";
      if (ssidName == wifi_ssid) options += " selected"; // Bôi đen sẵn WiFi cũ nếu trùng
      options += ">" + ssidName + " (" + String(WiFi.RSSI(i)) + "dBm)</option>";
    }
  }
  
  // Trộn dữ liệu vào HTML (Template Engine cơ bản)
  html.replace("%SSID_OPTIONS%", options);
  html.replace("%WIFI_PASS%", wifi_pass);
  html.replace("%FB_HOST%", fb_host);
  html.replace("%FB_AUTH%", fb_auth);
  
  server.send(200, "text/html", html);
}

void handleSave() {
  String new_ssid = server.arg("ssid");
  String new_pass = server.arg("pass");
  String new_host = server.arg("fb_host");
  String new_auth = server.arg("fb_auth");
  
  // Lưu vào bộ nhớ Flash
  saveConfig(new_ssid, new_pass, new_host, new_auth);
  
  // Phản hồi cho trình duyệt
  String successPage = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'><style>"
                       "body{font-family:'Inter',sans-serif;background:#0f172a;color:#f8fafc;display:flex;align-items:center;justify-content:center;height:100vh;margin:0;}"
                       ".box{background:rgba(255,255,255,0.05);padding:40px;border-radius:16px;text-align:center;border:1px solid rgba(255,255,255,0.1);box-shadow:0 10px 30px rgba(0,0,0,0.5);}"
                       "h2{color:#10b981;margin-top:0;}</style></head><body>"
                       "<div class='box'><h2>Đã lưu cấu hình thành công!</h2>"
                       "<p style='color:#94a3b8;line-height:1.6;'>Mạch ESP32 đang khởi động lại...<br>Vui lòng tắt WiFi này và chờ xe AGV kết nối vào mạng.</p></div>"
                       "</body></html>";
  server.send(200, "text/html", successPage);
  
  // Đợi 2 giây cho web kịp load rồi reset board
  delay(2000);
  ESP.restart();
}

void enterConfigMode() {
  isConfigMode = true;
  Serial.println("\n>>> VAO CHE DO CAU HINH WIFI & FIREBASE <<<");
  
  // Ngắt kết nối cũ và dọn dẹp bộ nhớ đệm WiFi để tránh kẹt module
  WiFi.disconnect(true, true);
  delay(500);

  // Cài đặt làm Trạm phát (Access Point) kèm khả năng quét (STA)
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  
  Serial.print("Da phat WiFi ten: "); Serial.println(AP_SSID);
  Serial.print("Mat khau WiFi   : "); Serial.println(AP_PASS);
  Serial.print("Truy cap Web qua IP: "); Serial.println(WiFi.softAPIP());
  
  // Khai báo đường dẫn Web
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
}

// ==========================================
// TASK ĐA LUỒNG: CHẠY FIREBASE TRÊN CORE 0
// ==========================================
void FirebaseTask(void *pvParameters) {
  static bool fb_connected = false;
  for (;;) {
    if (!isConfigMode && WiFi.status() == WL_CONNECTED) {
      if (millis() - lastCheckTime >= FIREBASE_CHECK_INTERVAL) {
        lastCheckTime = millis();

        if (Firebase.ready()) {
          if (!fb_connected) {
            Serial.println("[FIREBASE] Ket noi Server thanh cong!");
            fb_connected = true;
          }

          // Xử lý gửi trạng thái đã đến đích
          if (need_send_status) {
            if (Firebase.setInt(firebaseData, "/robot/camera_command/status", current_arrived_status)) {
              Serial.printf("[FIREBASE] Da gui trang thai den dich = %d\n", current_arrived_status);
              need_send_status = false;
            } else {
              Serial.printf("[FIREBASE] Loi gui trang thai: %s\n", firebaseData.errorReason().c_str());
            }
          }

          double newTimestamp = 0;
          if (Firebase.getDouble(firebaseData, "/robot/camera_command/timestamp")) {
            newTimestamp = firebaseData.doubleData();
          }

          if (Firebase.getString(firebaseData, "/robot/camera_command/command")) {
            if (firebaseData.dataType() == "string") {
              String newCommand = firebaseData.stringData();
              
              static bool isFirstFirebaseRead = true;
              
              if (isFirstFirebaseRead) {
                currentCommand = newCommand;
                currentTimestamp = newTimestamp;
                isFirstFirebaseRead = false;
                Serial.println("[FIREBASE] Bo qua lenh doc duoc lan dau tien luc khoi dong.");
              } 
              else if ((newCommand != currentCommand || newTimestamp != currentTimestamp) && newCommand != "") {
                currentCommand = newCommand;
                currentTimestamp = newTimestamp;
                isAckReceived = false;
                Serial.print("Nhan duoc lenh moi tu Firebase: ");
                Serial.println(currentCommand);
                
                // Bóc tách chuỗi NxxHyy
                int nIndex = currentCommand.indexOf("N");
                int hIndex = currentCommand.indexOf("H");
                
                if (nIndex != -1 && hIndex != -1 && hIndex > nIndex) {
                  String nStr = currentCommand.substring(nIndex + 1, hIndex);
                  String hStr = currentCommand.substring(hIndex + 1);
                  
                  firebase_node = (uint8_t)nStr.toInt();
                  firebase_h_cmd = (uint8_t)hStr.toInt();
                  last_firebase_cmd_time = 0; // Reset timer for new command
                  
                  Serial.printf("=> Node: %d, H_Cmd: %d\n", firebase_node, firebase_h_cmd);
                }
              }
            }
          } else {
            Serial.print("[FIREBASE] Loi doc data: ");
            Serial.println(firebaseData.errorReason());
          }
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Nhường CPU 10ms để Watchdog không reset
  }
}

// ==========================================
// CHƯƠNG TRÌNH CHÍNH
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUDRATE, SERIAL_8N1, RXD2, TXD2);
  HC12Serial.begin(HC12_BAUDRATE, SERIAL_8N1, HC12_RXD, HC12_TXD);
  
  // Khởi tạo I2C cho các cảm biến
  Wire.begin(I2C_SDA, I2C_SCL); 
  Wire.setClock(400000); // 400kHz cho VL53L5CX

  // Kích hoạt điện trở nội kéo lên (Pull-up) cho nút BOOT
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  delay(100); 

  // ==========================================
  // KHỞI TẠO CẢM BIẾN (NON-BLOCKING FAIL-SAFE)
  // ==========================================
  Serial.println("\n--- Kiem tra ket noi Cam bien ---");
  
  bno055_setup();
  bno055_setOperationModeNDOF();
  uint8_t bno_id = 0;
  bno055_readData(BNO055_CHIP_ID, &bno_id, 1);
  if (bno_id == BNO055_ID) {
    bno055_ok = true;
    Serial.println("[OK] BNO055 ket noi thanh cong!");
  } else {
    Serial.printf("[LOI] BNO055 khong phan hoi! (ID = 0x%02X)\n", bno_id);
  }

  Serial.println("Dang nap 90KB Firmware cho VL53L5CX (Se mat ~0.6s)...");
  if (myImager.begin()) {
    // Chuyển sang chế độ 4x4 để tăng tầm đo xa (lên đến 4m).
    // Chế độ 8x8 xử lý làm 4 khung hình phụ nên ở 10Hz max Integration chỉ được ~20ms, 
    // khiến cho xe bị "cận thị". Chuyển sang 4x4 cho phép Integration Time lên đến 90ms.
    myImager.setResolution(4 * 4); 
    myImager.setRangingFrequency(10); 
    myImager.setIntegrationTime(80); // Tăng lên 80ms để thu nhiều sáng hơn
    
    // THỬ SỬA BẰNG PHẦN MỀM: Bóp hẹp góc nhìn (FOV) của cảm biến lại để tia laser nhỏ hơn, lọt qua lỗ
    // Giá trị từ 0 đến 99%. Mặc định là 5%. Tăng lên 50% sẽ làm hẹp góc quét đáng kể.
    myImager.setSharpenerPercent(50); 
    
    myImager.startRanging();
    vl53_ok = true;
    Serial.println("[OK] VL53L5CX ket noi thanh cong! (Mode 4x4, 10Hz, Integration 80ms)");
  } else {
    Serial.println("[LOI] VL53L5CX khong phan hoi hoac loi nap Firmware!");
  }
  Serial.println("---------------------------------\n");

  // Lấy dữ liệu cũ từ bộ nhớ
  loadConfig();
  
  // Nếu chưa có cấu hình, dùng wifi mặc định để fallback
  if (wifi_ssid == "") {
    wifi_ssid = "du";
    wifi_pass = "1234##4321";
    Serial.println("[WIFI] Chua co cau hinh luu. Fallback sang 'du'.");
  }
  
  // KIỂM TRA NÚT BOOT: 
  // Nếu đè nút BOOT lúc cấp nguồn -> Trạng thái chân 0 sẽ là LOW
  if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
    Serial.println("Phat hien nut BOOT duoc nhan giu!");
    enterConfigMode();
  } else {
    Serial.print("Dang ket noi WiFi: "); Serial.println(wifi_ssid);
      
    WiFi.disconnect(true, true);
    delay(500);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.persistent(true);
    WiFi.setAutoReconnect(true);
    
    if (WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str()) != WL_CONNECTED) {
      // begin() không trả trạng thái kết nối ngay, chỉ khởi tạo
    }
    
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 25000) {
      Serial.print(".");
      delay(500);
    }
      
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\n[WIFI] Khong the ket noi WiFi. Tu dong chuyen sang che do cau hinh.");
      enterConfigMode();
    } else {
      Serial.println("\nWiFi da ket noi thanh cong!");
      Serial.print("IP Address: "); Serial.println(WiFi.localIP());
      Serial.println("[WIFI] Auto reconnect da duoc bat.");
        
      // 4. BẮT ĐẦU CHẠY FIREBASE
      // Làm sạch chuỗi fb_host và fb_auth (Tránh lỗi do user copy/paste dư khoảng trắng hoặc https://)
      fb_host.trim();
      if (fb_host.startsWith("https://")) fb_host = fb_host.substring(8);
      if (fb_host.startsWith("http://")) fb_host = fb_host.substring(7);
      if (fb_host.endsWith("/")) fb_host = fb_host.substring(0, fb_host.length() - 1);
      fb_auth.trim();
      
      config.database_url = fb_host.c_str();
      config.api_key = fb_auth.c_str();
      
      // Bật chế độ Test Mode (không cần đăng nhập Email/Pass) 
      // Rất quan trọng nếu chỉ dùng API Key với Realtime DB đang mở Rules
      config.signer.test_mode = true;
      
      // RẤT QUAN TRỌNG: Giảm kích thước bộ đệm SSL và Response
      // VL53L5CX đã ngốn gần hết RAM (90KB firmware). Nếu không giảm, mbedtls sẽ báo lỗi "Failed to initialize the SSL layer" do cạn kiệt Heap.
      firebaseData.setBSSLBufferSize(2048, 1024);
      firebaseData.setResponseSize(1024);

      Serial.printf("[MEM] Free Heap before Firebase: %u bytes\n", ESP.getFreeHeap());
      Serial.printf("[MEM] Max Free Block: %u bytes\n", ESP.getMaxAllocHeap());

      Firebase.begin(&config, &auth);
      Firebase.reconnectWiFi(true);
    }
  }

  // Khởi tạo Task Firebase chạy riêng biệt trên Core 0 (Core 1 để dành cho Loop chính xử lý RS485)
  xTaskCreatePinnedToCore(
    FirebaseTask,   // Hàm chạy Task
    "FirebaseTask", // Tên Task
    8192,           // Kích thước Stack (Firebase cần nhiều RAM)
    NULL,           // Tham số
    1,              // Độ ưu tiên
    &FirebaseTaskHandle, 
    0               // Chạy trên Core 0 (Core xử lý WiFi mặc định)
  );
}

void loop() {
  process_hc12_uart();
  // NẾU ĐANG Ở CHẾ ĐỘ CẤU HÌNH -> CHẠY THÊM WEBSERVER ĐỂ USER CẤU HÌNH
  if (isConfigMode) {
    server.handleClient();
  }

  // ==========================================
  // LOGIC RS485 & FIREBASE COMMAND
  // ==========================================
  // Đoạn đọc Firebase đã được chuyển sang Core 0 (FirebaseTask) để không làm kẹt vòng lặp này!
  
  // ==========================================
  // LOGIC ĐỌC IMU + VL53L5CX
  // ==========================================
  if (bno055_ok) {
    updateImuAccumulatedYaw();
  } else {
    current_yaw = 0.0f;
    imu_delta_yaw = 0.0f;
  }

  if (vl53_ok && myImager.isDataReady()) {
    myImager.getRangingData(&measurementData);

    uint16_t min_dist = 65534;
    const int vl53_zone_count = 16; // Chế độ 4x4 có 16 vùng
    for (int i = 0; i < vl53_zone_count; i++) {
      int row = i / 4;
      int col = i % 4;
      
      // Dựa vào ma trận thực tế: Góc trên bên trái bị vướng vỏ xe (ra số 8-10mm)
      // Góc dưới bên phải đã nhìn lọt qua lỗ (ra số 600-900mm).
      // Phần mềm sẽ "liếc" mắt sang phải và xuống dưới để lấy tín hiệu tốt nhất.
      if (row < 1 || col < 2) continue; // Chỉ lấy các cột 2,3 và hàng 1,2,3

      uint8_t st = measurementData.target_status[i];
      uint16_t dist = measurementData.distance_mm[i];
      // Target Status (Theo VL53L5CX):
      // 5: Range Valid, 9: Range Valid Merged Pulse, 
      // 10: Range Valid No Wraparound Check (Thường xuất hiện khi đo xa trên 1.5m)
      // 12: Target Blurred (Tín hiệu yếu, nhưng vẫn đo được khoảng cách)
      // Loại bỏ các khoảng cách < 50mm (do nhiễu kính bảo vệ/vỏ robot)
      if ((st == 5 || st == 9 || st == 10 || st == 12) && dist > 50 && dist < min_dist) {
        min_dist = dist;
      }
    }
    obstacle_distance = (min_dist == 65534) ? 65534 : min_dist;
  } else if (!vl53_ok) {
    obstacle_distance = 0xFFFF;
  }


  /*
  // CƠ CHẾ GỬI LẠI UART (CHỜ ACK TỪ STM32) - ĐÃ BỎ VÌ DÙNG KHUNG NHỊ PHÂN
  if (!isAckReceived && (millis() - lastSendTime >= 1000)) {
    lastSendTime = millis();
    Serial2.print(currentCommand);
    Serial2.print("\n");
    Serial.println("Dang gui lenh xuong STM32... (Cho ACK)");
  }
  */

  // TEST RS485: gửi frame liên tục để kiểm tra đèn RX/TX trên module UART-485
  /*
  if (millis() - lastRs485TestTxTime >= RS485_TEST_TX_INTERVAL_MS) {
    lastRs485TestTxTime = millis();

    int16_t yaw_int = 0x7FFF;
    if (bno055_ok) {
      updateImuAccumulatedYaw();
      yaw_int = (int16_t)(imu_total_yaw * 10.0f);
      if (yaw_int == 0x7FFF) yaw_int = 0x7FFE;
    }

    uint8_t tx_frame[11];
    tx_frame[0] = 0xAA; tx_frame[1] = 0x55;
    tx_frame[2] = RS485_ADDR; tx_frame[3] = 0x01;
    tx_frame[4] = (yaw_int >> 8) & 0xFF;
    tx_frame[5] = yaw_int & 0xFF;
    tx_frame[6] = firebase_node;
    tx_frame[7] = firebase_h_cmd;
    tx_frame[8] = (obstacle_distance >> 8) & 0xFF;
    tx_frame[9] = obstacle_distance & 0xFF;
    tx_frame[10] = calculate_checksum(tx_frame, 2, 9);

    Serial2.write(tx_frame, 11);

    Serial.printf("[TEST-TX] yaw=%.1fdeg node=%u cmd=%u obs=%u | ",
                  imu_total_yaw,
                  (unsigned)firebase_node,
                  (unsigned)firebase_h_cmd,
                  (unsigned)obstacle_distance);
    for (int i = 0; i < 11; i++) {
      Serial.printf("%02X ", tx_frame[i]);
    }
    Serial.println();
  }
  */

  // ==========================================
  // XỬ LÝ NHẬN DATA RS485 (MASTER-SLAVE + FIREBASE ACK)
  // ==========================================
  process_main_uart();
}
