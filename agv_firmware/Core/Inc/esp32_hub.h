#ifndef __ESP32_HUB_H
#define __ESP32_HUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

#define ESP32_ADDR 0x63  // 99
#define ESP32_CMD_READ_IMU 0x01
#define ESP32_CMD_ARM_TEXT 0x02
#define ESP32_MAX_ARM_CMD_LEN 48

// Cấu trúc lưu trữ dữ liệu từ ESP32
typedef struct {
    float Yaw;
    uint32_t LastUpdateTick;
    bool IsConnected;
    uint8_t TargetNode;
    uint8_t H_Command;
    bool HasNewCommand;
    uint16_t ObstacleDistance;
    bool HasNewArmCommand;
    char ArmCommand[ESP32_MAX_ARM_CMD_LEN + 1];
} ESP32_SensorData_t;

// Khai báo biến toàn cục
extern ESP32_SensorData_t esp32_data;
extern uint8_t esp32_rx_buffer[64];

// Các hàm API
void ESP32_Init(UART_HandleTypeDef *huart);
void ESP32_RequestData(uint16_t current_node, uint8_t is_arrived);
void ESP32_ParseResponse(uint16_t length);
ESP32_SensorData_t ESP32_GetSafeData(void);

#ifdef __cplusplus
}
#endif

#endif /* __ESP32_HUB_H */
