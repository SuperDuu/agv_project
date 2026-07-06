#include "esp32_hub.h"
#include <stdio.h>
#include <string.h>

ESP32_SensorData_t esp32_data = {0};
uint8_t esp32_rx_buffer[64];

volatile uint32_t dbg_rx_success = 0;
volatile uint32_t dbg_rx_bad_len = 0;
volatile uint32_t dbg_rx_bad_cs = 0;
volatile uint32_t dbg_rx_sync_lost = 0;

static UART_HandleTypeDef *esp32_huart;

static uint8_t esp32_frame_buffer[AGV_PROTO_V2_MAX_FRAME_LEN];
static uint16_t esp32_frame_index = 0;
static uint16_t esp32_expected_len = 0;
static uint8_t esp32_frame_state = 0;
static uint8_t esp32_seq_counter = 0;

static uint16_t AGV_ProtoV2_Crc16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFFu;

  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000u) {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static void ESP32_ResetParser(void) {
  esp32_frame_state = 0;
  esp32_frame_index = 0;
  esp32_expected_len = 0;
}

static void ESP32_FormatLegacyArmCommand(const AGV_ProtoV2_ArmJointCommand_t *cmd) {
  char prefix = (cmd->arm_id == AGV_PROTO_V2_ARM_RIGHT_ID) ? 'R' : 'L';

  int q1 = (cmd->q1_x100 >= 0) ? (cmd->q1_x100 + 50) / 100 : (cmd->q1_x100 - 50) / 100;
  int q2 = (cmd->q2_x100 >= 0) ? (cmd->q2_x100 + 50) / 100 : (cmd->q2_x100 - 50) / 100;
  int q3 = (cmd->q3_x100 >= 0) ? (cmd->q3_x100 + 50) / 100 : (cmd->q3_x100 - 50) / 100;
  int q4 = (cmd->q4_x100 >= 0) ? (cmd->q4_x100 + 50) / 100 : (cmd->q4_x100 - 50) / 100;
  int q5 = (cmd->q5_x100 >= 0) ? (cmd->q5_x100 + 50) / 100 : (cmd->q5_x100 - 50) / 100;
  int q6 = (cmd->q6_x100 >= 0) ? (cmd->q6_x100 + 50) / 100 : (cmd->q6_x100 - 50) / 100;

  snprintf(esp32_data.ArmCommand, sizeof(esp32_data.ArmCommand), "%c:%d,%d,%d,%d,%d,%d",
           prefix, q1, q2, q3, q4, q5, q6);
  esp32_data.HasNewArmCommand = true;
}

static void ESP32_ProcessSensorReport(const uint8_t *payload, uint16_t payload_len) {
  if (payload_len != 8u) {
    dbg_rx_bad_len++;
    return;
  }

  int16_t yaw_x100 = AGV_ProtoV2_ReadS16BE(&payload[0]);
  uint8_t sensor_flags = payload[7];

  if (sensor_flags & AGV_PROTO_V2_SENSOR_FLAG_IMU_VALID) {
    esp32_data.Yaw = (float)yaw_x100 / 100.0f;
  } else {
    esp32_data.Yaw = 65535.0f;
  }

  if (sensor_flags & AGV_PROTO_V2_SENSOR_FLAG_VL53_VALID) {
    esp32_data.ObstacleDistance = AGV_ProtoV2_ReadU16BE(&payload[2]);
  } else {
    esp32_data.ObstacleDistance = 0xFFFFu;
  }

  if (sensor_flags & AGV_PROTO_V2_SENSOR_FLAG_NEW_TARGET) {
    esp32_data.TargetNode = (uint8_t)AGV_ProtoV2_ReadU16BE(&payload[4]);
    esp32_data.H_Command = payload[6];
    esp32_data.HasNewCommand = true;
  }

  esp32_data.LastUpdateTick = HAL_GetTick();
  esp32_data.IsConnected = true;
  dbg_rx_success++;
}

static void ESP32_ProcessArmJointCommand(const uint8_t *payload, uint16_t payload_len) {
  if (payload_len != 18u) {
    dbg_rx_bad_len++;
    return;
  }

  AGV_ProtoV2_ArmJointCommand_t cmd;
  cmd.arm_id = payload[0];
  cmd.motion_mode = payload[1];
  cmd.q1_x100 = AGV_ProtoV2_ReadS16BE(&payload[2]);
  cmd.q2_x100 = AGV_ProtoV2_ReadS16BE(&payload[4]);
  cmd.q3_x100 = AGV_ProtoV2_ReadS16BE(&payload[6]);
  cmd.q4_x100 = AGV_ProtoV2_ReadS16BE(&payload[8]);
  cmd.q5_x100 = AGV_ProtoV2_ReadS16BE(&payload[10]);
  cmd.q6_x100 = AGV_ProtoV2_ReadS16BE(&payload[12]);
  cmd.move_time_ms = AGV_ProtoV2_ReadU16BE(&payload[14]);
  cmd.arm_flags = payload[16];
  cmd.reserved = payload[17];

  if (cmd.motion_mode != AGV_PROTO_V2_MOTION_ABSOLUTE &&
      cmd.motion_mode != AGV_PROTO_V2_MOTION_HOME &&
      cmd.motion_mode != AGV_PROTO_V2_MOTION_ESTOP) {
    dbg_rx_bad_len++;
    return;
  }

  ESP32_FormatLegacyArmCommand(&cmd);
  dbg_rx_success++;
}

static void ESP32_ProcessFrame(const uint8_t *frame, uint16_t frame_len) {
  if (frame_len < AGV_PROTO_V2_FRAME_OVERHEAD ||
      frame[0] != AGV_PROTO_V2_SOF1 || frame[1] != AGV_PROTO_V2_SOF2) {
    dbg_rx_sync_lost++;
    return;
  }

  uint16_t payload_len = AGV_ProtoV2_ReadU16BE(&frame[6]);
  if ((uint16_t)(payload_len + AGV_PROTO_V2_FRAME_OVERHEAD) != frame_len) {
    dbg_rx_bad_len++;
    return;
  }

  if (frame[2] != AGV_PROTO_V2_ADDR_MAIN) {
    return;
  }

  uint16_t rx_crc = AGV_ProtoV2_ReadU16BE(&frame[8 + payload_len]);
  uint16_t calc_crc = AGV_ProtoV2_Crc16(&frame[2], (uint16_t)(6 + payload_len));
  if (rx_crc != calc_crc) {
    dbg_rx_bad_cs++;
    return;
  }

  switch (frame[4]) {
  case AGV_PROTO_V2_CMD_SENSOR_REPORT:
    ESP32_ProcessSensorReport(&frame[8], payload_len);
    break;

  case AGV_PROTO_V2_CMD_ARM_JOINT_COMMAND:
    ESP32_ProcessArmJointCommand(&frame[8], payload_len);
    break;

  default:
    dbg_rx_sync_lost++;
    break;
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

  uint8_t tx_frame[AGV_PROTO_V2_FRAME_OVERHEAD + sizeof(AGV_ProtoV2_SyncRequest_t)];
  AGV_ProtoV2_SyncRequest_t sync_req;
  sync_req.current_node = current_node;
  sync_req.is_arrived = is_arrived;
  sync_req.reserved = 0;

  tx_frame[0] = AGV_PROTO_V2_SOF1;
  tx_frame[1] = AGV_PROTO_V2_SOF2;
  tx_frame[2] = AGV_PROTO_V2_ADDR_ESP32;
  tx_frame[3] = AGV_PROTO_V2_ADDR_MAIN;
  tx_frame[4] = AGV_PROTO_V2_CMD_SYNC_REQUEST;
  tx_frame[5] = esp32_seq_counter++;
  AGV_ProtoV2_WriteU16BE(&tx_frame[6], 4u);
  AGV_ProtoV2_WriteU16BE(&tx_frame[8], sync_req.current_node);
  tx_frame[10] = sync_req.is_arrived;
  tx_frame[11] = 0;

  uint16_t crc = AGV_ProtoV2_Crc16(&tx_frame[2], 10u);
  AGV_ProtoV2_WriteU16BE(&tx_frame[12], crc);

  HAL_StatusTypeDef status = HAL_UART_Transmit(esp32_huart, tx_frame, sizeof(tx_frame), 20);
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
      if (b == AGV_PROTO_V2_SOF1) {
        esp32_frame_buffer[0] = b;
        esp32_frame_index = 1;
        esp32_frame_state = 1;
      }
      break;

    case 1:
      if (b == AGV_PROTO_V2_SOF2) {
        esp32_frame_buffer[1] = b;
        esp32_frame_index = 2;
        esp32_frame_state = 2;
      } else if (b == AGV_PROTO_V2_SOF1) {
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

      if (esp32_frame_index == 8u) {
        uint16_t payload_len = AGV_ProtoV2_ReadU16BE(&esp32_frame_buffer[6]);
        if (payload_len > AGV_PROTO_V2_MAX_PAYLOAD_LEN) {
          dbg_rx_bad_len++;
          ESP32_ResetParser();
          break;
        }
        esp32_expected_len = (uint16_t)(AGV_PROTO_V2_FRAME_OVERHEAD + payload_len);
      }

      if (esp32_expected_len != 0u && esp32_frame_index == esp32_expected_len) {
        ESP32_ProcessFrame(esp32_frame_buffer, esp32_expected_len);
        ESP32_ResetParser();
      }
      break;

    default:
      ESP32_ResetParser();
      break;
    }
  }

  if (HAL_GetTick() - esp32_data.LastUpdateTick > 1000u) {
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
