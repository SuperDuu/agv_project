#include "qr50_reader.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  Khởi tạo thiết bị đầu đọc QR50
 * @param  handler: Con trỏ tới struct quản lý QR50
 * @param  huart: Con trỏ tới ngoại vi UART/RS485
 * @param  addr: Địa chỉ thiết bị (0 là quảng bá)
 */
void QR50_Init(QR50_Handler_t *handler, UART_HandleTypeDef *huart,
               uint8_t addr) {
  if (handler == NULL || huart == NULL)
    return;

  // Khởi tạo ngoại vi
  handler->huart = huart;

  // Khởi tạo cấu hình mặc định theo tài liệu
  handler->Config.Address = addr;
  handler->Config.Baudrate = 115200;
  handler->Config.ActiveUpload = true; // Mặc định ở chế độ tự động đẩy dữ liệu

  // Khóa cờ dữ liệu mới và reset buffer
  handler->Data.New_Data_Flag = false;
  handler->Data.Data_Length = 0;
  memset(handler->Data.Data_Buffer, 0, QR50_MAX_DATA_LEN);

  // Trạng thái ban đầu
  handler->Status = QR50_OK;
}

/**
 * @brief  Phân tích gói tin thô từ UART (gọi trong ngắt IDLE)
 * @param  handler: Con trỏ tới struct quản lý QR50
 * @param  raw_buffer: Mảng chứa dữ liệu nhận được từ RS485
 * @param  length: Số lượng byte nhận được
 */
void QR50_ParseData(QR50_Handler_t *handler, uint8_t *raw_buffer,
                    uint16_t length) {
  if (handler == NULL || raw_buffer == NULL || length == 0)
    return;

  // Kiểm tra độ dài tránh tràn mảng
  if (length > QR50_MAX_DATA_LEN) {
    length = QR50_MAX_DATA_LEN;
    handler->Status = QR50_ERROR;
  } else {
    handler->Status = QR50_OK;
  }

  // Copy dữ liệu thô vào Buffer của thiết bị (chỉ copy nếu dùng 2 buffer khác
  // nhau)
  if (raw_buffer != handler->Data.Data_Buffer) {
    memcpy(handler->Data.Data_Buffer, raw_buffer, length);
  }
  handler->Data.Data_Length = length;

  // Đảm bảo kết thúc chuỗi (để in ra màn hình dễ dàng nếu là mã QR string)
  if (length < QR50_MAX_DATA_LEN) {
    handler->Data.Data_Buffer[length] = '\0';
  }

  // Bật cờ báo hiệu đã có dữ liệu mới để vòng lặp chính (main loop) xử lý
  handler->Data.New_Data_Flag = true;

  /* USER CODE BEGIN ParseData */
  // Có thể bổ sung thêm logic lọc Header, Checksum hoặc bóc tách riêng ID Thẻ /
  // Mã QR ở đây Tùy thuộc vào datasheet định dạng khung truyền của QR50.
  /* USER CODE END ParseData */
}

/* USER CODE BEGIN 1 */
void Wiegand_Init(Wiegand_HandleTypeDef *hwg) {
  if (hwg == NULL)
    return;
  hwg->bit_count = 0;
  memset(hwg->raw_bits, 0, sizeof(hwg->raw_bits));
  hwg->last_bit_time = 0;
  hwg->new_data_ready = false;
  hwg->final_card_id = 0;
}

// Gọi hàm này trong ngắt EXTI Falling Edge
void Wiegand_ProcessBit(Wiegand_HandleTypeDef *hwg, uint8_t bit) {
  if (hwg == NULL)
    return;

  uint32_t now = HAL_GetTick();

  // Nếu khoảng thời gian giữa 2 bit > 50ms, coi như bắt đầu quẹt thẻ mới
  if (now - hwg->last_bit_time > 50) {
    hwg->bit_count = 0;
    memset(hwg->raw_bits, 0, sizeof(hwg->raw_bits));
  }

  hwg->last_bit_time = now;

  // Lưu bit mới vào mảng
  if (hwg->bit_count < 128) {
    hwg->raw_bits[hwg->bit_count] = bit;
    hwg->bit_count++;
  }
}

// Gọi hàm này trong vòng lặp while(1)
void Wiegand_ProcessLoop(Wiegand_HandleTypeDef *hwg) {
  if (hwg == NULL)
    return;

  uint32_t now = HAL_GetTick();

  // Nếu đã nhận đủ bit và sau 50ms không có bit mới nào đến
  if (hwg->bit_count > 0 && (now - hwg->last_bit_time > 50)) {

    // --- WIEGAND 66 ---
    // QR50BM WG66: 1 bit parity + 64 bit data + 1 bit parity = 66 bits
    if (hwg->bit_count == 66) {
      uint64_t raw_data = 0;
      for (int i = 0; i < 64; i++) {
        raw_data = (raw_data << 1) | hwg->raw_bits[1 + i];
      }
      
      // Lưu lại raw_data vào biến debug_wiegand_raw (64 bit) để Live Expression
      extern volatile uint64_t debug_wiegand_raw;
      debug_wiegand_raw = raw_data;

      // Không XOR nữa, bóc tách riêng 32 bit cao và 32 bit thấp để xem
      uint32_t part1 = (raw_data >> 32) & 0xFFFFFFFF;
      uint32_t part2 = raw_data & 0xFFFFFFFF;
      
      extern volatile uint32_t debug_wiegand_high32;
      extern volatile uint32_t debug_wiegand_low32;
      debug_wiegand_high32 = part1;
      debug_wiegand_low32 = part2;

      hwg->final_card_id = part2;
      hwg->new_data_ready = true;
    } 
    // --- WIEGAND 34 ---
    else if (hwg->bit_count == 34) {
      uint32_t card_data = 0;
      for (int i = 0; i < 32; i++) {
        card_data = (card_data << 1) | hwg->raw_bits[1 + i];
      }
      
      extern volatile uint64_t debug_wiegand_raw;
      debug_wiegand_raw = card_data;

      hwg->final_card_id = card_data;
      hwg->new_data_ready = true;
    } 
    // --- WIEGAND 26 ---
    else if (hwg->bit_count == 26) {
      uint32_t card_data = 0;
      for (int i = 0; i < 24; i++) {
        card_data = (card_data << 1) | hwg->raw_bits[1 + i];
      }
      
      extern volatile uint64_t debug_wiegand_raw;
      debug_wiegand_raw = card_data;

      // Reverse output cho WG26
      uint32_t reversed_card = 0;
      reversed_card |= (card_data & 0xFF0000) >> 16;
      reversed_card |= (card_data & 0x00FF00);
      reversed_card |= (card_data & 0x0000FF) << 16;
      
      hwg->final_card_id = reversed_card;
      hwg->new_data_ready = true;
    } else {
      // Số bit không xác định
      hwg->final_card_id = 0xFFFFFFFF;
      hwg->new_data_ready = true;
    }

    // Lưu lại số lượng bit vào biến toàn cục để theo dõi trên Live Expression
    extern volatile uint32_t debug_wiegand_bit_count;
    debug_wiegand_bit_count = hwg->bit_count;

    // Reset sau khi xử lý xong
    hwg->bit_count = 0;
    memset(hwg->raw_bits, 0, sizeof(hwg->raw_bits));
  }
}
/* USER CODE END 1 */
