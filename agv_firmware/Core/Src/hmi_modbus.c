#include "hmi_modbus.h"
#include "agv_control.h"
#include "agv_routing.h"
#include "esp32_hub.h"
#include <string.h>

uint16_t hmi_registers[HMI_REG_COUNT] = {0};
HMI_HandleTypeDef h_hmi;

// Biến lưu trạng thái cũ để bắt sự kiện thay đổi
static uint16_t prev_dest_node = 0xFFFF;
static uint16_t prev_command = 0;

// Các biến từ main.c / agv_control.c
extern uint16_t current_path[];
extern uint16_t path_length;
extern AGV_State_t agv_state;
extern volatile bool hmi_tx_in_progress;
extern void Route_Recalculate(void); // Cần định nghĩa hoặc dùng flag trong main

// Tính mã CRC16 cho Modbus RTU
static uint16_t Modbus_CalcCRC(uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// Bộ đệm xử lý tách biệt khỏi DMA
static uint8_t process_buffer[256];
static volatile uint16_t process_len = 0;
static volatile bool need_restart_dma = false;

// Khởi tạo UART DMA/IT cho Modbus
void HMI_Init(UART_HandleTypeDef *huart, uint8_t slave_address) {
  h_hmi.huart = huart;
  h_hmi.slave_address = slave_address;
  h_hmi.rx_index = 0;
  h_hmi.frame_ready = false;

  // Đồng bộ các biến mặc định lên HMI khi khởi tạo để tránh bị ghi đè về 0
  hmi_registers[REG_AGV_MODE] = agv_state.run_mode;
  hmi_registers[REG_DEST_NODE] = agv_state.destination_node;
  hmi_registers[REG_CURRENT_NODE] = agv_state.current_node;

  prev_dest_node =
      agv_state
          .destination_node; // Đồng bộ để không tự tính đường đi khi khởi động
  prev_command = 0;

  // Kích hoạt ngắt toàn cục UART để bắt sự kiện IDLE LINE
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  // Khởi động nhận DMA
  HAL_UARTEx_ReceiveToIdle_DMA(h_hmi.huart, h_hmi.rx_buffer,
                               sizeof(h_hmi.rx_buffer));
}

// Callback khi UART nhận được dữ liệu (Gọi từ HAL_UARTEx_RxEventCallback trong
// main)
void HMI_RxCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == h_hmi.huart->Instance) {
    // Nháy đèn SYS_LED1 báo hiệu nhận được dữ liệu
    HAL_GPIO_TogglePin(SYS_LED1_GPIO_Port, SYS_LED1_Pin);

    // Copy dữ liệu sang bộ đệm xử lý ngay lập tức (trong ngắt, trước khi DMA
    // ghi đè)
    if (Size <= sizeof(process_buffer)) {
      memcpy(process_buffer, h_hmi.rx_buffer, Size);
      process_len = Size;
      h_hmi.frame_ready = true;
    }

    // Đánh dấu cần khởi động lại DMA (sẽ thực hiện trong HMI_Process ở main
    // loop)
    need_restart_dma = true;
  }
}

// Khởi động lại DMA an toàn (chỉ dùng khi cần, không gọi liên tục trong main
// loop)
void HMI_RestartDMA(void) {
  if (!need_restart_dma)
    return;
  need_restart_dma = false;

  // Dừng hoàn toàn DMA cũ
  HAL_UART_AbortReceive(h_hmi.huart);

  // Xóa tất cả cờ lỗi UART
  __HAL_UART_CLEAR_FLAG(h_hmi.huart,
                        UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);

  // Khởi động lại DMA từ đầu (con trỏ quay về rx_buffer[0])
  HAL_UARTEx_ReceiveToIdle_DMA(h_hmi.huart, h_hmi.rx_buffer,
                               sizeof(h_hmi.rx_buffer));
}

// Xử lý gói tin Modbus (Gọi liên tục trong while(1))
void HMI_Process(void) {
  if (!h_hmi.frame_ready) {
    HMI_RestartDMA();
    return;
  }
  h_hmi.frame_ready = false;

  uint16_t len = process_len;
  if (len < 8)
    goto end_process; // Khung Modbus RTU tối thiểu 8 bytes

  // Kiểm tra Slave Address
  if (process_buffer[0] != h_hmi.slave_address)
    goto end_process;

  // Kiểm tra CRC
  uint16_t received_crc =
      (process_buffer[len - 1] << 8) | process_buffer[len - 2];
  uint16_t calc_crc = Modbus_CalcCRC(process_buffer, len - 2);
  if (received_crc != calc_crc) {
    extern volatile uint32_t debug_hmi_crc_err;
    debug_hmi_crc_err++;
    goto end_process;
  }

  uint8_t function_code = process_buffer[1];
  uint16_t start_addr = (process_buffer[2] << 8) | process_buffer[3];
  uint16_t reg_count = (process_buffer[4] << 8) | process_buffer[5];

  // Chống ghi đè ngoài mảng
  if (start_addr + reg_count > HMI_REG_COUNT)
    goto end_process;

  uint16_t tx_len = 0;

  // Lệnh 0x03: Đọc thanh ghi Holding (Read Holding Registers)
  if (function_code == 0x03) {
    h_hmi.tx_buffer[0] = h_hmi.slave_address;
    h_hmi.tx_buffer[1] = 0x03;
    h_hmi.tx_buffer[2] = reg_count * 2; // Số byte trả về

    uint16_t byte_idx = 3;
    for (uint16_t i = 0; i < reg_count; i++) {
      h_hmi.tx_buffer[byte_idx++] = (hmi_registers[start_addr + i] >> 8) & 0xFF;
      h_hmi.tx_buffer[byte_idx++] = hmi_registers[start_addr + i] & 0xFF;
    }

    uint16_t crc = Modbus_CalcCRC(h_hmi.tx_buffer, byte_idx);
    h_hmi.tx_buffer[byte_idx++] = crc & 0xFF;
    h_hmi.tx_buffer[byte_idx++] = (crc >> 8) & 0xFF;

    tx_len = byte_idx;
  }
  // Lệnh 0x06: Ghi 1 thanh ghi (Write Single Register)
  else if (function_code == 0x06) {
    uint16_t reg_value = (process_buffer[4] << 8) | process_buffer[5];
    hmi_registers[start_addr] = reg_value;

    // Echo lại toàn bộ lệnh 0x06
    memcpy(h_hmi.tx_buffer, process_buffer, len);
    tx_len = len;
  }
  // Lệnh 0x10: Ghi nhiều thanh ghi (Write Multiple Registers)
  else if (function_code == 0x10) {
    // HMI gửi function 16, đính kèm số lượng byte data = reg_count * 2
    // Data bắt đầu từ byte số 7
    uint8_t byte_count = process_buffer[6];
    if (byte_count != reg_count * 2)
      goto end_process; // Kiểm tra tính hợp lệ
    if (7 + byte_count > len - 2)
      goto end_process; // Tránh đọc lố mảng

    for (uint16_t i = 0; i < reg_count; i++) {
      hmi_registers[start_addr + i] =
          (process_buffer[7 + i * 2] << 8) | process_buffer[8 + i * 2];
    }

    // Trả lời lệnh 0x10
    h_hmi.tx_buffer[0] = h_hmi.slave_address;
    h_hmi.tx_buffer[1] = 0x10;
    h_hmi.tx_buffer[2] = process_buffer[2];
    h_hmi.tx_buffer[3] = process_buffer[3];
    h_hmi.tx_buffer[4] = process_buffer[4];
    h_hmi.tx_buffer[5] = process_buffer[5];

    uint16_t crc = Modbus_CalcCRC(h_hmi.tx_buffer, 6);
    h_hmi.tx_buffer[6] = crc & 0xFF;
    h_hmi.tx_buffer[7] = (crc >> 8) & 0xFF;
    tx_len = 8;
  }

  if (tx_len > 0) {
    // === Bảo vệ: Cấm ErrorCallback can thiệp DMA trong quá trình TX ===
    extern volatile bool hmi_tx_in_progress;
    hmi_tx_in_progress = true;

    // Tắt DMA RX trước khi truyền (RS485 half-duplex)
    HAL_UART_AbortReceive(h_hmi.huart);

    // Ép trạng thái UART TX về READY
    h_hmi.huart->gState = HAL_UART_STATE_READY;
    __HAL_UNLOCK(h_hmi.huart);

    // RS485 Turnaround Delay
    HAL_Delay(3);

    // Truyền dữ liệu
    HAL_UART_Transmit(h_hmi.huart, h_hmi.tx_buffer, tx_len, 100);

    // HAL_UART_Transmit đã tự động đợi cờ TC (Transmission Complete) hoặc
    // Timeout. Xóa vòng lặp while(TC) vô tận ở đây để tránh treo hệ thống nếu
    // cáp đứt/nhiễu.

    // Chờ bus RS485 ổn định sau khi truyền
    HAL_Delay(2);

    // Xóa toàn bộ lỗi và dữ liệu echo còn sót trong RX
    __HAL_UART_CLEAR_FLAG(h_hmi.huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                           UART_CLEAR_FEF | UART_CLEAR_IDLEF);
    __HAL_UART_SEND_REQ(h_hmi.huart, UART_RXDATA_FLUSH_REQUEST);

    // Khởi động lại DMA RX
    HAL_UARTEx_ReceiveToIdle_DMA(h_hmi.huart, h_hmi.rx_buffer,
                                 sizeof(h_hmi.rx_buffer));
    need_restart_dma = false;

    // === Gỡ bảo vệ: Cho phép ErrorCallback hoạt động bình thường ===
    hmi_tx_in_progress = false;

    // Nháy LED3 báo hiệu truyền xong
    HAL_GPIO_TogglePin(SYS_LED3_GPIO_Port, SYS_LED3_Pin);
  }

end_process:
  // Đảm bảo luôn luôn khởi động lại DMA nếu chưa được restart
  // (Bảo vệ HMI không bị "điếc" khi nhận phải gói tin lỗi CRC)
  HMI_RestartDMA();
}

// Đồng bộ dữ liệu HMI với hệ thống
void HMI_SyncData(void) {
  // 1. STM32 -> HMI (Cập nhật dữ liệu từ biến vào mảng)
  hmi_registers[REG_AGV_MODE] = agv_state.run_mode;
  hmi_registers[REG_CURRENT_NODE] = agv_state.current_node;
  hmi_registers[REG_PATH_LENGTH] = path_length;

  // Thêm: Điểm bắt đầu (Start Point) và Điểm tiếp theo (Next Point)
  hmi_registers[REG_NEXT_NODE] = current_path[agv_state.path_index];
  // hmi_registers[REG_DEST_NODE] đã được lấy từ HMI xuống nên không cần ghi đè
  // ngược lại

  // Cập nhật trạng thái chạy của AGV (0: Idle, 1: Running, 2: Error)
  if (agv_state.indicator_state == 3) {
    hmi_registers[REG_AGV_STATUS] = 2; // HMI Error
  } else if (agv_state.follow_line_enable) {
    hmi_registers[REG_AGV_STATUS] = 1; // HMI Running
  } else {
    hmi_registers[REG_AGV_STATUS] = 0; // HMI Idle
  }

  // Cập nhật đèn trạng thái kết nối ESP32 (1: Có tín hiệu, 0: Bị lỗi/Chưa nhận
  // được)
  ESP32_SensorData_t esp_data = ESP32_GetSafeData();
  bool esp_connected = (HAL_GetTick() - esp_data.LastUpdateTick < 1000) &&
                       (esp_data.LastUpdateTick > 0);

  if (esp_connected) {
    hmi_registers[REG_INDICATOR] = 1; // Đã có tín hiệu
  } else {
    hmi_registers[REG_INDICATOR] = 0; // Mất tín hiệu hoặc chưa nhận được
  }

  if (path_length > 0) {
    hmi_registers[REG_PATH_START] = current_path[0];
  } else {
    hmi_registers[REG_PATH_START] = 0;
  }

  // 2. HMI -> STM32 (Lắng nghe lệnh từ màn hình)
  if (hmi_registers[REG_DEST_NODE] != prev_dest_node) {
    prev_dest_node = hmi_registers[REG_DEST_NODE];
    agv_state.destination_node = prev_dest_node;

    // Đặt cờ để tính toán lại quỹ đạo trong hàm while(1) của main.c
    agv_state.need_recalculate_path = true;
  }

  if (hmi_registers[REG_COMMAND] != prev_command) {
    prev_command = hmi_registers[REG_COMMAND];
    if (prev_command == 1) {
      agv_state.follow_line_enable = true; // Chạy
      agv_state.indicator_state = 0;       // Xóa lỗi

      // Nháy LED1 để báo hiệu đã nhận lệnh START từ màn hình
      HAL_GPIO_TogglePin(SYS_LED1_GPIO_Port, SYS_LED1_Pin);

      agv_state.last_leave_intersection_time =
          HAL_GetTick(); // Reset blind zone để không bị dừng ngay lập tức
      agv_state.last_qr_time =
          HAL_GetTick(); // Reset watchdog QR để xe không bị tắt ngay
    } else if (prev_command == 2) {
      agv_state.follow_line_enable = false; // Dừng
    } else if (prev_command == 3) {
      agv_state.indicator_state = 0; // Xóa lỗi
    }
    hmi_registers[REG_COMMAND] = 0; // Xóa lệnh sau khi thực hiện
    prev_command = 0;
  }
}
