#include "esp32_hub.h"
#include <string.h>

ESP32_SensorData_t esp32_data = {0};
uint8_t esp32_rx_buffer[64];

volatile uint32_t dbg_rx_success = 0;
volatile uint32_t dbg_rx_bad_len = 0;
volatile uint32_t dbg_rx_bad_cs = 0;
volatile uint32_t dbg_rx_sync_lost = 0;

static UART_HandleTypeDef *esp32_huart;

static uint8_t esp32_frame_buffer[64];
static uint8_t esp32_frame_state = 0;
static uint8_t esp32_frame_index = 0;
static uint8_t esp32_expected_len = 0;

static uint8_t Calculate_Checksum(const uint8_t *data, uint8_t start, uint8_t end) {
    uint8_t cs = 0;
    for (uint8_t i = start; i <= end; i++) {
        cs ^= data[i];
    }
    return cs;
}

static void ESP32_ResetParser(void) {
    esp32_frame_state = 0;
    esp32_frame_index = 0;
    esp32_expected_len = 0;
}

static void ESP32_ProcessImuFrame(const uint8_t *frame) {
    uint8_t calc_cs = Calculate_Checksum(frame, 2, 9);
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

static void ESP32_ProcessArmFrame(const uint8_t *frame) {
    uint8_t payload_len = frame[4];

    if (payload_len < 3 || payload_len > ESP32_MAX_ARM_CMD_LEN) {
        dbg_rx_bad_len++;
        return;
    }

    uint8_t calc_cs = Calculate_Checksum(frame, 2, (uint8_t)(4 + payload_len));
    if (calc_cs != frame[5 + payload_len]) {
        dbg_rx_bad_cs++;
        return;
    }

    if ((frame[5] != 'L' && frame[5] != 'R') || frame[6] != ':') {
        dbg_rx_bad_len++;
        return;
    }

    memcpy(esp32_data.ArmCommand, &frame[5], payload_len);
    esp32_data.ArmCommand[payload_len] = '\0';
    esp32_data.HasNewArmCommand = true;
}

static void ESP32_ProcessFrame(const uint8_t *frame, uint8_t frame_len) {
    if (frame_len < 5 || frame[0] != 0xAA || frame[1] != 0x55 || frame[2] != ESP32_ADDR) {
        dbg_rx_sync_lost++;
        return;
    }

    switch (frame[3]) {
    case ESP32_CMD_READ_IMU:
        if (frame_len != 11) {
            dbg_rx_bad_len++;
            return;
        }
        ESP32_ProcessImuFrame(frame);
        return;

    case ESP32_CMD_ARM_TEXT:
        ESP32_ProcessArmFrame(frame);
        return;

    default:
        dbg_rx_sync_lost++;
        return;
    }
}

void ESP32_Init(UART_HandleTypeDef *huart) {
    esp32_huart = huart;
    memset(&esp32_data, 0, sizeof(esp32_data));
    esp32_data.ObstacleDistance = 0xFFFF;
    ESP32_ResetParser();

    HAL_UART_AbortReceive(esp32_huart);
    __HAL_UART_CLEAR_FLAG(esp32_huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                           UART_CLEAR_FEF | UART_CLEAR_PEF);
    HAL_UARTEx_ReceiveToIdle_DMA(esp32_huart, esp32_rx_buffer, sizeof(esp32_rx_buffer));
}

void ESP32_RequestData(uint16_t current_node, uint8_t is_arrived) {
    if (esp32_huart == NULL) {
        return;
    }

    uint8_t tx_frame[8];
    tx_frame[0] = 0xAA;
    tx_frame[1] = 0x55;
    tx_frame[2] = ESP32_ADDR;
    tx_frame[3] = ESP32_CMD_READ_IMU;
    tx_frame[4] = (current_node >> 8) & 0xFF;
    tx_frame[5] = current_node & 0xFF;
    tx_frame[6] = is_arrived;
    tx_frame[7] = Calculate_Checksum(tx_frame, 2, 6);

    HAL_StatusTypeDef status = HAL_UART_Transmit(esp32_huart, tx_frame, 8, 10);
    if (status != HAL_OK) {
        esp32_huart->gState = HAL_UART_STATE_READY;
        __HAL_UNLOCK(esp32_huart);
    }
}

void ESP32_ParseResponse(uint16_t length) {
    if (length > sizeof(esp32_rx_buffer)) {
        length = sizeof(esp32_rx_buffer);
    }

    for (uint16_t i = 0; i < length; i++) {
        uint8_t b = esp32_rx_buffer[i];

        switch (esp32_frame_state) {
        case 0:
            if (b == 0xAA) {
                esp32_frame_buffer[0] = b;
                esp32_frame_index = 1;
                esp32_frame_state = 1;
            }
            break;

        case 1:
            if (b == 0x55) {
                esp32_frame_buffer[1] = b;
                esp32_frame_index = 2;
                esp32_frame_state = 2;
            } else if (b == 0xAA) {
                esp32_frame_buffer[0] = b;
                esp32_frame_index = 1;
            } else {
                ESP32_ResetParser();
            }
            break;

        case 2:
            if (esp32_frame_index >= sizeof(esp32_frame_buffer)) {
                dbg_rx_bad_len++;
                ESP32_ResetParser();
                break;
            }

            esp32_frame_buffer[esp32_frame_index++] = b;

            if (esp32_frame_index == 4) {
                if (esp32_frame_buffer[3] == ESP32_CMD_READ_IMU) {
                    esp32_expected_len = 11;
                } else if (esp32_frame_buffer[3] == ESP32_CMD_ARM_TEXT) {
                    esp32_expected_len = 0;
                } else {
                    dbg_rx_sync_lost++;
                    ESP32_ResetParser();
                    break;
                }
            }

            if (esp32_frame_index == 5 && esp32_frame_buffer[3] == ESP32_CMD_ARM_TEXT) {
                uint8_t payload_len = esp32_frame_buffer[4];
                if (payload_len == 0 || payload_len > ESP32_MAX_ARM_CMD_LEN) {
                    dbg_rx_bad_len++;
                    ESP32_ResetParser();
                    break;
                }
                esp32_expected_len = (uint8_t)(6 + payload_len);
            }

            if (esp32_expected_len != 0 && esp32_frame_index == esp32_expected_len) {
                ESP32_ProcessFrame(esp32_frame_buffer, esp32_expected_len);
                ESP32_ResetParser();
            }
            break;

        default:
            ESP32_ResetParser();
            break;
        }
    }

    if (HAL_GetTick() - esp32_data.LastUpdateTick > 1000) {
        esp32_data.IsConnected = false;
    }
}

ESP32_SensorData_t ESP32_GetSafeData(void) {
    ESP32_SensorData_t copy;
    __disable_irq();
    copy = esp32_data;
    __enable_irq();
    return copy;
}
