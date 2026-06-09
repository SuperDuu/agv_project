#include "ls7366r.h"

// Biến lưu trữ giá trị counter trước đó để tính toán vận tốc
static int32_t prev_counter[4] = {0, 0, 0, 0};

/**
 * @brief  Tạo delay ngắn khoảng 50ns để đáp ứng timing của LS7366R.
 *         Với STM32H5 (250MHz), mỗi chu kỳ máy là 4ns.
 */
static void delay_50ns(void) {
    // 15 NOPs x 4ns = 60ns
    for (volatile int i = 0; i < 15; i++) {
        __asm("nop");
    }
}

/**
 * @brief Lấy Port và Pin tương ứng với ID của Encoder (1 đến 4).
 */
static void Get_CS_Pin(uint8_t csPin, GPIO_TypeDef **port, uint16_t *pin) {
    switch (csPin) {
        case ENC_CS1:
            *port = T_ENC_CS1_GPIO_Port;
            *pin = T_ENC_CS1_Pin;
            break;
        case ENC_CS2:
            *port = T_ENC_CS2_GPIO_Port;
            *pin = T_ENC_CS2_Pin;
            break;
        case ENC_CS3:
            *port = T_ENC_CS3_GPIO_Port;
            *pin = T_ENC_CS3_Pin;
            break;
        case ENC_CS4:
            *port = T_ENC_CS4_GPIO_Port;
            *pin = T_ENC_CS4_Pin;
            break;
        default:
            *port = NULL;
            *pin = 0;
            break;
    }
}

/**
 * @brief Kéo chân CS xuống mức LOW để bắt đầu giao tiếp SPI.
 */
static void CS_Low(uint8_t csPin) {
    GPIO_TypeDef *port;
    uint16_t pin;
    Get_CS_Pin(csPin, &port, &pin);
    if (port != NULL) {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        delay_50ns(); // Đảm bảo t_CSS (CS setup time) >= 50ns
    }
}

/**
 * @brief Kéo chân CS lên mức HIGH để kết thúc giao tiếp SPI.
 */
static void CS_High(uint8_t csPin) {
    GPIO_TypeDef *port;
    uint16_t pin;
    Get_CS_Pin(csPin, &port, &pin);
    if (port != NULL) {
        delay_50ns(); // Đảm bảo data hold time
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        delay_50ns(); // Đảm bảo t_CSW (CS disable time) >= 50ns
    }
}

/**
 * @brief Gửi và nhận 1 byte qua SPI.
 */
static uint8_t SPI_Transfer(uint8_t data) {
    uint8_t rxData = 0;
    HAL_SPI_TransmitReceive(LS7366R_SPI_HANDLE, &data, &rxData, 1, HAL_MAX_DELAY);
    return rxData;
}

/**
 * @brief Khởi tạo 1 chip LS7366R với cấu hình cho MDR0 và MDR1.
 */
void LS7366R_Init(uint8_t csPin, uint8_t mdr0, uint8_t mdr1) {
    // Đảm bảo CS ở mức HIGH trước khi bắt đầu
    CS_High(csPin);

    // Xóa counter và status register
    LS7366R_ClearCounter(csPin);
    
    CS_Low(csPin);
    SPI_Transfer(LS_CLEAR_STR);
    CS_High(csPin);

    // Ghi cấu hình vào MDR0
    CS_Low(csPin);
    SPI_Transfer(LS_WRITE_MDR0);
    SPI_Transfer(mdr0);
    CS_High(csPin);

    // Ghi cấu hình vào MDR1
    CS_Low(csPin);
    SPI_Transfer(LS_WRITE_MDR1);
    SPI_Transfer(mdr1);
    CS_High(csPin);
}

/**
 * @brief Khởi tạo đồng loạt cả 4 chip LS7366R với thông số mặc định (Hall encoder).
 */
void LS7366R_InitAll(void) {
    // Kéo toàn bộ chân CS lên HIGH trước tiên để tránh xung đột SPI (Do cấu hình mặc định IOC là LOW)
    CS_High(ENC_CS1);
    CS_High(ENC_CS2);
    CS_High(ENC_CS3);
    CS_High(ENC_CS4);

    for (uint8_t i = 1; i <= 4; i++) {
        LS7366R_Init(i, LS_MDR0_QUAD_X4_FREE, LS_MDR1_4BYTE_EN);
        prev_counter[i - 1] = 0;
    }
}

/**
 * @brief Đọc trực tiếp giá trị Counter (CNTR) của chip.
 */
int32_t LS7366R_ReadCounter(uint8_t csPin) {
    int32_t count = 0;
    
    // Ngắt toàn bộ interrupts để tránh bị ngắt SPI giữa chừng gây hỏng frame
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    CS_Low(csPin);
    SPI_Transfer(LS_READ_CNTR);
    count |= ((uint32_t)SPI_Transfer(0x00) << 24);
    count |= ((uint32_t)SPI_Transfer(0x00) << 16);
    count |= ((uint32_t)SPI_Transfer(0x00) << 8);
    count |= ((uint32_t)SPI_Transfer(0x00));
    CS_High(csPin);

    __set_PRIMASK(primask);

    return count;
}

/**
 * @brief Xóa trắng giá trị Counter về 0.
 */
void LS7366R_ClearCounter(uint8_t csPin) {
    CS_Low(csPin);
    SPI_Transfer(LS_CLEAR_CNTR);
    CS_High(csPin);
}

/**
 * @brief Đọc thanh ghi trạng thái (STR).
 */
uint8_t LS7366R_ReadStatus(uint8_t csPin) {
    uint8_t status = 0;
    
    CS_Low(csPin);
    SPI_Transfer(LS_READ_STR);
    status = SPI_Transfer(0x00);
    CS_High(csPin);
    
    return status;
}

/**
 * @brief Copy giá trị từ CNTR sang OTR (latch). Dùng để đọc đồng thời mà không bị nhiễu do xung mới.
 */
void LS7366R_LoadOTR(uint8_t csPin) {
    CS_Low(csPin);
    SPI_Transfer(LS_LOAD_OTR);
    CS_High(csPin);
}

/**
 * @brief Đọc giá trị đã được latch trong thanh ghi OTR.
 */
int32_t LS7366R_ReadOTR(uint8_t csPin) {
    int32_t count = 0;
    
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    CS_Low(csPin);
    SPI_Transfer(LS_READ_OTR);
    count |= ((uint32_t)SPI_Transfer(0x00) << 24);
    count |= ((uint32_t)SPI_Transfer(0x00) << 16);
    count |= ((uint32_t)SPI_Transfer(0x00) << 8);
    count |= ((uint32_t)SPI_Transfer(0x00));
    CS_High(csPin);

    __set_PRIMASK(primask);

    return count;
}

/**
 * @brief Tính toán vận tốc (Counts per second) và tự động xử lý tràn số (rollover).
 */
float LS7366R_GetVelocity(uint8_t csPin, float dt_seconds) {
    if (csPin < 1 || csPin > 4 || dt_seconds <= 0.0f) {
        return 0.0f;
    }

    // Đọc nguyên tử thông qua OTR để đảm bảo tính chính xác tại thời điểm gọi
    LS7366R_LoadOTR(csPin);
    int32_t current_counter = LS7366R_ReadOTR(csPin);

    // Tính delta (tự động xử lý bù trừ tràn 32-bit nhờ đặc tính của kiểu int32_t)
    int32_t delta = current_counter - prev_counter[csPin - 1];
    prev_counter[csPin - 1] = current_counter;

    // Tính tốc độ
    float velocity = (float)delta / dt_seconds;
    
    return velocity;
}
