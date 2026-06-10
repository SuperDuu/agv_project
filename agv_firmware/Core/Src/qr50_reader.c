#include "qr50_reader.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Khởi tạo thiết bị đầu đọc QR50
  * @param  handler: Con trỏ tới struct quản lý QR50
  * @param  huart: Con trỏ tới ngoại vi UART/RS485
  * @param  addr: Địa chỉ thiết bị (0 là quảng bá)
  */
void QR50_Init(QR50_Handler_t *handler, UART_HandleTypeDef *huart, uint8_t addr)
{
    if (handler == NULL || huart == NULL) return;
    
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
void QR50_ParseData(QR50_Handler_t *handler, uint8_t *raw_buffer, uint16_t length)
{
    if (handler == NULL || raw_buffer == NULL || length == 0) return;
    
    // Kiểm tra độ dài tránh tràn mảng
    if (length > QR50_MAX_DATA_LEN) {
        length = QR50_MAX_DATA_LEN;
        handler->Status = QR50_ERROR;
    } else {
        handler->Status = QR50_OK;
    }
    
    // Copy dữ liệu thô vào Buffer của thiết bị (chỉ copy nếu dùng 2 buffer khác nhau)
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
    // Có thể bổ sung thêm logic lọc Header, Checksum hoặc bóc tách riêng ID Thẻ / Mã QR ở đây
    // Tùy thuộc vào datasheet định dạng khung truyền của QR50.
    /* USER CODE END ParseData */
}

/* USER CODE BEGIN 1 */
void Wiegand_Init(Wiegand_HandleTypeDef *hwg) {
    if (hwg == NULL) return;
    hwg->bit_count = 0;
    hwg->data_buffer = 0;
    hwg->last_bit_time = 0;
    hwg->new_data_ready = false;
    hwg->final_card_id = 0;
}

// Gọi hàm này trong ngắt EXTI Falling Edge
void Wiegand_ProcessBit(Wiegand_HandleTypeDef *hwg, uint8_t bit) {
    if (hwg == NULL) return;
    
    uint32_t now = HAL_GetTick();
    
    // Nếu khoảng thời gian giữa 2 bit > 50ms, coi như bắt đầu quẹt thẻ mới
    if (now - hwg->last_bit_time > 50) {
        hwg->bit_count = 0;
        hwg->data_buffer = 0;
    }
    
    hwg->last_bit_time = now;
    
    // Dịch bit và gán bit mới
    hwg->data_buffer <<= 1;
    hwg->data_buffer |= bit;
    hwg->bit_count++;
}

// Gọi hàm này trong vòng lặp while(1)
void Wiegand_ProcessLoop(Wiegand_HandleTypeDef *hwg) {
    if (hwg == NULL) return;
    
    uint32_t now = HAL_GetTick();
    
    // Nếu đã nhận đủ bit và sau 50ms không có bit mới nào đến
    if (hwg->bit_count > 0 && (now - hwg->last_bit_time > 50)) {
        
        // Chuẩn Wiegand 34 (reverse hoặc thuận)
        if (hwg->bit_count == 34) {
            uint8_t p_even = (hwg->data_buffer >> 33) & 0x01; // Bit 33 (đầu tiên)
            uint8_t p_odd = hwg->data_buffer & 0x01;          // Bit 0 (cuối cùng)
            uint32_t card_data = (hwg->data_buffer >> 1) & 0xFFFFFFFF;
            
            // Tính chẵn lẻ (Parity Check)
            uint8_t count_even = 0;
            uint8_t count_odd = 0;
            uint32_t first_16 = (card_data >> 16) & 0xFFFF;
            uint32_t last_16 = card_data & 0xFFFF;
            
            for (int i = 0; i < 16; i++) {
                if ((first_16 >> i) & 1) count_even++;
                if ((last_16 >> i) & 1) count_odd++;
            }
            
            // Even Parity: Tổng số bit 1 của (p_even + first_16) phải là số chẵn
            // Odd Parity: Tổng số bit 1 của (p_odd + last_16) phải là số lẻ
            bool parity_ok = (((count_even + p_even) % 2) == 0) && (((count_odd + p_odd) % 2) != 0);
            
            if (parity_ok) {
                // Bỏ tính năng đảo byte (Reverse output) để xem số gốc
                hwg->final_card_id = card_data;
                hwg->new_data_ready = true;
            } else {
                // Lỗi Parity
                hwg->final_card_id = 0xFFFFFFFF; // Báo mã lỗi
                hwg->new_data_ready = true;
            }
        } else if (hwg->bit_count == 26) {
            uint32_t card_data = (hwg->data_buffer >> 1) & 0xFFFFFF;
            
            uint32_t reversed_card = 0;
            reversed_card |= (card_data & 0xFF0000) >> 16;
            reversed_card |= (card_data & 0x00FF00);
            reversed_card |= (card_data & 0x0000FF) << 16;
            
            hwg->final_card_id = reversed_card;
            hwg->new_data_ready = true;
        } else {
            // Số bit không xác định, có thể nhiễu, lưu tạm để debug
            hwg->final_card_id = (uint32_t)hwg->data_buffer;
            hwg->new_data_ready = true;
        }
        
        // Lưu lại số lượng bit vào biến toàn cục để theo dõi trên Live Expression
        extern volatile uint32_t debug_wiegand_bit_count;
        debug_wiegand_bit_count = hwg->bit_count;
        
        // Reset sau khi xử lý xong
        hwg->bit_count = 0;
        hwg->data_buffer = 0;
    }
}
/* USER CODE END 1 */
