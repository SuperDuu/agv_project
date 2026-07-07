#ifndef __ESP32_HUB_H
#define __ESP32_HUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "agv_link_protocol.h"
#include <stdbool.h>

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

extern volatile uint32_t dbg_rx_success;
extern volatile uint32_t dbg_rx_bad_len;
extern volatile uint32_t dbg_rx_bad_cs;
extern volatile uint32_t dbg_rx_sync_lost;
extern volatile uint32_t dbg_rx_bad_addr;
extern volatile uint32_t dbg_rx_unsupported_cmd;
extern volatile uint32_t dbg_arm_seen;
extern volatile uint32_t dbg_arm_ok;
extern volatile uint32_t dbg_arm_bad_len;
extern volatile uint32_t dbg_arm_bad_mode;

extern volatile uint8_t  dbg_rx_last_dest;
extern volatile uint8_t  dbg_rx_last_src;
extern volatile uint8_t  dbg_rx_last_cmd;
extern volatile uint8_t  dbg_rx_last_seq;
extern volatile uint16_t dbg_rx_last_payload_len;
extern volatile uint16_t dbg_rx_last_frame_len;
extern volatile uint16_t dbg_rx_last_crc_rx;
extern volatile uint16_t dbg_rx_last_crc_calc;
extern volatile uint8_t  dbg_arm_last_dest;
extern volatile uint8_t  dbg_arm_last_motion_mode;
extern volatile uint8_t  dbg_arm_last_flags;
extern volatile uint16_t dbg_arm_last_payload_len;

// Các hàm API
void ESP32_Init(UART_HandleTypeDef *huart);
void ESP32_RequestData(uint16_t current_node, uint8_t is_arrived);
void ESP32_ParseResponse(uint16_t length);
ESP32_SensorData_t ESP32_GetSafeData(void);

#ifdef __cplusplus
}
#endif

#endif /* __ESP32_HUB_H */
