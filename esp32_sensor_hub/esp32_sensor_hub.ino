#include <Arduino.h>

// TODO: Sau này bạn thêm thư viện BNO055 và VL53L5CX vào đây
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>

#define RS485_ADDR      0x63  // 99 theo hệ thập phân
#define RS485_BAUD      115200

// Cấu hình chân UART2 cho module RS485 (Tùy thuộc board ESP32 của bạn)
#define RS485_TX_PIN    17
#define RS485_RX_PIN    16

// Biến lưu trữ Node hiện tại nhận từ STM32
uint16_t current_node = 0xFFFF;
float current_yaw = 0.0f; // Góc mô phỏng

// Bộ đệm nhận
uint8_t rx_buffer[10];
uint8_t rx_index = 0;
uint32_t last_rx_time = 0;

void setup() {
  Serial.begin(115200); // Debug qua cổng USB
  Serial.println("ESP32 Sensor Hub Khởi động!");
  
  // Khởi tạo Serial2 kết nối với module RS485 (MAX3485 Auto Direction)
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  // TODO: Khởi tạo I2C và BNO055 ở đây
}

// Hàm tính XOR Checksum
uint8_t calculate_checksum(uint8_t *data, uint8_t start, uint8_t end) {
  uint8_t cs = 0;
  for (uint8_t i = start; i <= end; i++) {
    cs ^= data[i];
  }
  return cs;
}

// Xử lý gói tin nhận được từ STM32
void parse_rs485_frame() {
  // Gói hợp lệ từ STM32 là 7 bytes
  if (rx_index == 7) {
    // Check Header
    if (rx_buffer[0] == 0xAA && rx_buffer[1] == 0x55) {
      // Check Address
      if (rx_buffer[2] == RS485_ADDR) {
        // Check Checksum (Byte 6 = XOR từ Byte 2 đến Byte 5)
        uint8_t calc_cs = calculate_checksum(rx_buffer, 2, 5);
        if (calc_cs == rx_buffer[6]) {
          
          // Lệnh 0x01: STM32 Gửi Node, yêu cầu ESP32 trả về Yaw
          if (rx_buffer[3] == 0x01) {
            current_node = (rx_buffer[4] << 8) | rx_buffer[5];
            Serial.printf("[RS485] Nhan current_node: %d\n", current_node);

            // TODO: Ở đây bạn đọc góc thực tế từ BNO055
            // current_yaw = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();
            current_yaw += 0.5f; // Chạy giả lập góc
            if (current_yaw > 360.0f) current_yaw -= 360.0f;

            // Đóng gói trả lời STM32 (7 bytes)
            uint8_t tx_frame[7];
            tx_frame[0] = 0xAA;
            tx_frame[1] = 0x55;
            tx_frame[2] = RS485_ADDR;
            tx_frame[3] = 0x01;
            
            // Ép kiểu góc Yaw (float) thành int16 (nhân 10) để bảo toàn 1 số thập phân
            int16_t yaw_int = (int16_t)(current_yaw * 10.0f);
            tx_frame[4] = (yaw_int >> 8) & 0xFF; // MSB
            tx_frame[5] = yaw_int & 0xFF;        // LSB
            
            tx_frame[6] = calculate_checksum(tx_frame, 2, 5);

            // Phản hồi về STM32
            Serial2.write(tx_frame, 7);
          }
        } else {
          Serial.println("[RS485] Loi Checksum!");
        }
      }
    }
  }
  
  // Xóa đệm chuẩn bị nhận gói mới
  rx_index = 0;
}

void loop() {
  // Lắng nghe cổng RS485 liên tục
  while (Serial2.available() > 0) {
    uint8_t b = Serial2.read();
    
    // Nếu quá 10ms không có byte mới -> bị đứt gói -> Reset đệm
    if (millis() - last_rx_time > 10) {
      rx_index = 0;
    }
    
    if (rx_index < sizeof(rx_buffer)) {
      rx_buffer[rx_index++] = b;
    }
    
    last_rx_time = millis();
    
    // Gói của chúng ta fix cứng 7 byte
    if (rx_index == 7) {
      parse_rs485_frame();
    }
  }

  // TODO: Tác vụ đưa Firebase chạy ngầm (hoặc chạy ở Task riêng trên Core 0)
}
