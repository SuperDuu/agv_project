#include "esp32_hub.h"
#include <string.h>

ESP32_SensorData_t esp32_data = {0};
uint8_t esp32_rx_buffer[15];

static UART_HandleTypeDef *esp32_huart;

// Tính XOR Checksum
static uint8_t Calculate_Checksum(uint8_t *data, uint8_t start, uint8_t end) {
    uint8_t cs = 0;
    for (uint8_t i = start; i <= end; i++) {
        cs ^= data[i];
    }
    return cs;
}

// Khởi tạo và khởi động DMA
void ESP32_Init(UART_HandleTypeDef *huart) {
    esp32_huart = huart;
    esp32_data.IsConnected = false;
    esp32_data.Yaw = 0.0f;
    esp32_data.LastUpdateTick = 0;
    esp32_data.HasNewCommand = false;

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

    HAL_UART_Transmit(esp32_huart, tx_frame, 7, 10); // Truyền nhanh, timeout 10ms
}

// Được gọi trong ngắt HAL_UARTEx_RxEventCallback khi nhận xong 1 frame từ ESP32
void ESP32_ParseResponse(uint16_t length) {
    // Khung truyền cố định 11 byte
    if (length == 11) {
        if (esp32_rx_buffer[0] == 0xAA && esp32_rx_buffer[1] == 0x55) {
            if (esp32_rx_buffer[2] == ESP32_ADDR && esp32_rx_buffer[3] == ESP32_CMD_READ_IMU) {
                uint8_t calc_cs = Calculate_Checksum(esp32_rx_buffer, 2, 9);
                if (calc_cs == esp32_rx_buffer[10]) {
                    // Dữ liệu hợp lệ
                    int16_t yaw_int = (esp32_rx_buffer[4] << 8) | esp32_rx_buffer[5];
                    if (yaw_int == (int16_t)0xFFFF) {
                        esp32_data.Yaw = 0xFFFF; // Cờ báo lỗi BNO055
                    } else {
                        esp32_data.Yaw = (float)yaw_int / 10.0f;
                    }
                    
                    if (esp32_rx_buffer[6] != 255) {
                        esp32_data.TargetNode = esp32_rx_buffer[6];
                        esp32_data.H_Command = esp32_rx_buffer[7];
                        esp32_data.HasNewCommand = true;
                    }
                    
                    esp32_data.ObstacleDistance = (esp32_rx_buffer[8] << 8) | esp32_rx_buffer[9];
                    
                    esp32_data.LastUpdateTick = HAL_GetTick();
                    esp32_data.IsConnected = true;
                }
            }
        }
    }
    
    // Cảnh báo mất kết nối nếu quá 1 giây không nhận được gói hợp lệ
    if (HAL_GetTick() - esp32_data.LastUpdateTick > 1000) {
        esp32_data.IsConnected = false;
    }

    // Luôn luôn khởi động lại DMA để đón gói tiếp theo
    HAL_UARTEx_ReceiveToIdle_DMA(esp32_huart, esp32_rx_buffer, sizeof(esp32_rx_buffer));
}
