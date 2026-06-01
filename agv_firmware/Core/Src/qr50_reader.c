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
    
    // Copy dữ liệu thô vào Buffer của thiết bị
    memcpy(handler->Data.Data_Buffer, raw_buffer, length);
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

/* USER CODE END 1 */
