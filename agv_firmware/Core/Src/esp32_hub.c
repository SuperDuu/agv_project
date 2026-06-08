#include "esp32_hub.h"
#include <string.h>

ESP32_SensorData_t esp32_data = {0};
uint8_t esp32_rx_buffer[15];

// Biến đếm Debug (xem trên Live Expressions)
volatile uint32_t dbg_rx_success = 0;
volatile uint32_t dbg_rx_bad_len = 0;
volatile uint32_t dbg_rx_bad_cs = 0;
volatile uint32_t dbg_rx_sync_lost = 0;

static UART_HandleTypeDef *esp32_huart;

// State machine ghép frame theo byte stream để tránh lỗi ReceiveToIdle trả về frame ngắn
static uint8_t esp32_frame_buffer[11];
static uint8_t esp32_frame_state = 0;   // 0=wait AA, 1=wait 55, 2=collect rest
static uint8_t esp32_frame_index = 0;   // số byte đã có trong frame_buffer

// Tính XOR Checksum
static uint8_t Calculate_Checksum(uint8_t *data, uint8_t start, uint8_t end) {
    uint8_t cs = 0;
    for (uint8_t i = start; i <= end; i++) {
        cs ^= data[i];
    }
    return cs;
}

static void ESP32_ProcessFrame(const uint8_t *frame) {
    if (frame[0] != 0xAA || frame[1] != 0x55) {
        dbg_rx_sync_lost++;
        return;
    }

    if (frame[2] != ESP32_ADDR || frame[3] != ESP32_CMD_READ_IMU) {
        dbg_rx_sync_lost++;
        return;
    }

    uint8_t calc_cs = Calculate_Checksum((uint8_t *)frame, 2, 9);
    if (calc_cs != frame[10]) {
        dbg_rx_bad_cs++;
        return;
    }

    dbg_rx_success++;
    int16_t yaw_int = (int16_t)((frame[4] << 8) | frame[5]);
    if (yaw_int == 0x7FFF) {
        esp32_data.Yaw = 65535.0f;
    } else {
        esp32_data.Yaw = (float)yaw_int / 10.0f;
    }

    if (frame[6] != 255) {
        esp32_data.TargetNode = frame[6];
        esp32_data.H_Command = frame[7];
        esp32_data.HasNewCommand = true;
    }

    esp32_data.ObstacleDistance = (uint16_t)((frame[8] << 8) | frame[9]);
    esp32_data.LastUpdateTick = HAL_GetTick();
    esp32_data.IsConnected = true;
}

// Khởi tạo và khởi động DMA
void ESP32_Init(UART_HandleTypeDef *huart) {
    esp32_huart = huart;
    esp32_data.IsConnected = false;
    esp32_data.Yaw = 0.0f;
    esp32_data.LastUpdateTick = 0;
    esp32_data.HasNewCommand = false;
    esp32_data.TargetNode = 0;
    esp32_data.H_Command = 0;
    esp32_data.ObstacleDistance = 0xFFFF;
    esp32_frame_index = 0;
    esp32_frame_state = 0;

    // Đảm bảo DMA/UART ở trạng thái sạch trước khi nhận
    HAL_UART_AbortReceive(esp32_huart);
    __HAL_UART_CLEAR_FLAG(esp32_huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

    // Kích hoạt DMA nhận chuỗi
    HAL_UARTEx_ReceiveToIdle_DMA(esp32_huart, esp32_rx_buffer, sizeof(esp32_rx_buffer));
}

// STM32 (Master) yêu cầu dữ liệu từ ESP32, đồng thời báo Node hiện tại
void ESP32_RequestData(uint16_t current_node) {
    if (esp32_huart == NULL) return;

    uint8_t tx_frame[7];
    tx_frame[0] = 0xAA;
    tx_frame[1] = 0x55;
    tx_frame[2] = ESP32_ADDR;
    tx_frame[3] = ESP32_CMD_READ_IMU;
    tx_frame[4] = (current_node >> 8) & 0xFF;
    tx_frame[5] = current_node & 0xFF;
    tx_frame[6] = Calculate_Checksum(tx_frame, 2, 5);

    HAL_StatusTypeDef status = HAL_UART_Transmit(esp32_huart, tx_frame, 7, 10);
    if (status != HAL_OK) {
        esp32_huart->gState = HAL_UART_STATE_READY;
        __HAL_UNLOCK(esp32_huart);
    }
}

// Ghép frame bằng state machine byte-by-byte, tối ưu cho gói 11 byte ngắn
void ESP32_ParseResponse(uint16_t length) {
    if (length > sizeof(esp32_rx_buffer)) {
        length = sizeof(esp32_rx_buffer);
    }

    for (uint16_t i = 0; i < length; i++) {
        uint8_t b = esp32_rx_buffer[i];

        switch (esp32_frame_state) {
            case 0: // wait 0xAA
                if (b == 0xAA) {
                    esp32_frame_buffer[0] = b;
                    esp32_frame_index = 1;
                    esp32_frame_state = 1;
                }
                break;

            case 1: // wait 0x55
                if (b == 0x55) {
                    esp32_frame_buffer[1] = b;
                    esp32_frame_index = 2;
                    esp32_frame_state = 2;
                } else if (b == 0xAA) {
                    esp32_frame_buffer[0] = b;
                    esp32_frame_index = 1;
                    esp32_frame_state = 1;
                } else {
                    esp32_frame_state = 0;
                    esp32_frame_index = 0;
                }
                break;

            case 2: // collect remaining 9 bytes
                esp32_frame_buffer[esp32_frame_index++] = b;
                if (esp32_frame_index == 11) {
                    ESP32_ProcessFrame(esp32_frame_buffer);
                    esp32_frame_state = 0;
                    esp32_frame_index = 0;
                }
                break;

            default:
                esp32_frame_state = 0;
                esp32_frame_index = 0;
                break;
        }
    }

    if (HAL_GetTick() - esp32_data.LastUpdateTick > 1000) {
        esp32_data.IsConnected = false;
    }

    // Không restart DMA ở đây để tránh trùng callback/treo UART; DMA đã được giữ chạy liên tục
}

ESP32_SensorData_t ESP32_GetSafeData(void) {
    ESP32_SensorData_t copy;
    __disable_irq();
    copy = esp32_data;
    __enable_irq();
    return copy;
}
