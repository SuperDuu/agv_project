/* ==========================================================================
 * esp32_hub.c  —  AGV/ARM Protocol V2.1 parser (AGV main side)
 *
 * Frame layout (V2.1):
 *   [SOF1][SOF2][DEST][SRC][LEN_L][LEN_H][CMD][SEQ][PAYLOAD...][CRC_L][CRC_H]
 *     0     1     2    3     4      5      6    7     8..        -2    -1
 *
 * All multi-byte fields: Little-Endian.
 * LEN = payload byte count (does NOT include header or CRC).
 * CRC covers bytes 2..8+N-1 (DEST through last PAYLOAD byte).
 * ========================================================================== */

#include "esp32_hub.h"
#include <stdio.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * Public data
 * -------------------------------------------------------------------------- */
ESP32_SensorData_t esp32_data = {0};
uint8_t esp32_rx_buffer[64];

volatile uint32_t dbg_rx_success    = 0;
volatile uint32_t dbg_rx_bad_len    = 0;
volatile uint32_t dbg_rx_bad_cs     = 0;
volatile uint32_t dbg_rx_sync_lost  = 0;
volatile uint32_t dbg_rx_bad_addr   = 0;
volatile uint32_t dbg_rx_unsupported_cmd = 0;
volatile uint32_t dbg_arm_seen      = 0;
volatile uint32_t dbg_arm_ok        = 0;
volatile uint32_t dbg_arm_bad_len   = 0;
volatile uint32_t dbg_arm_bad_mode  = 0;

volatile uint8_t  dbg_rx_last_dest        = 0;
volatile uint8_t  dbg_rx_last_src         = 0;
volatile uint8_t  dbg_rx_last_cmd         = 0;
volatile uint8_t  dbg_rx_last_seq         = 0;
volatile uint16_t dbg_rx_last_payload_len = 0;
volatile uint16_t dbg_rx_last_frame_len   = 0;
volatile uint16_t dbg_rx_last_crc_rx      = 0;
volatile uint16_t dbg_rx_last_crc_calc    = 0;
volatile uint8_t  dbg_arm_last_dest        = 0;
volatile uint8_t  dbg_arm_last_motion_mode = 0;
volatile uint8_t  dbg_arm_last_flags       = 0;
volatile uint16_t dbg_arm_last_payload_len = 0;

/* --------------------------------------------------------------------------
 * Private state
 * -------------------------------------------------------------------------- */
static UART_HandleTypeDef *esp32_huart;

static uint8_t  esp32_frame_buffer[AGV_PROTO_V2_MAX_FRAME_LEN];
static uint16_t esp32_frame_index   = 0;
static uint16_t esp32_expected_len  = 0;   /* total frame bytes expected      */
static uint8_t  esp32_frame_state   = 0;
static uint8_t  esp32_seq_counter   = 0;

/* --------------------------------------------------------------------------
 * CRC-16/CCITT-FALSE  (poly 0x1021, init 0xFFFF, no reflection)
 * -------------------------------------------------------------------------- */
static uint16_t AGV_ProtoV2_Crc16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFFu;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/* --------------------------------------------------------------------------
 * Reset state machine
 * -------------------------------------------------------------------------- */
static void ESP32_ResetParser(void) {
  esp32_frame_state = 0;
  esp32_frame_index = 0;
  esp32_expected_len = 0;
}

/* --------------------------------------------------------------------------
 * Legacy text bridge — formats V2.1 joint data as "R:q1,q2,q3,q4,q5,q6\n"
 * for the arm slave firmware that still uses the old text parser.
 * dest: AGV_PROTO_V2_ADDR_ARM_LEFT (0x02) or AGV_PROTO_V2_ADDR_ARM_RIGHT (0x03)
 * -------------------------------------------------------------------------- */
static void ESP32_FormatLegacyArmCommand(uint8_t dest,
                                         const AGV_ProtoV2_ArmJointCommand_t *cmd) {
  char prefix = (dest == AGV_PROTO_V2_ADDR_ARM_RIGHT) ? 'R' : 'L';
  char *arm_command = (dest == AGV_PROTO_V2_ADDR_ARM_RIGHT) ?
                      esp32_data.ArmCommandRight :
                      esp32_data.ArmCommandLeft;
  bool *has_new_command = (dest == AGV_PROTO_V2_ADDR_ARM_RIGHT) ?
                          &esp32_data.HasNewArmCommandRight :
                          &esp32_data.HasNewArmCommandLeft;

  /* Convert x100 fixed-point → integer degree (round to nearest) */
  int q1 = (cmd->q1_x100 >= 0) ? (cmd->q1_x100 + 50) / 100 : (cmd->q1_x100 - 50) / 100;
  int q2 = (cmd->q2_x100 >= 0) ? (cmd->q2_x100 + 50) / 100 : (cmd->q2_x100 - 50) / 100;
  int q3 = (cmd->q3_x100 >= 0) ? (cmd->q3_x100 + 50) / 100 : (cmd->q3_x100 - 50) / 100;
  int q4 = (cmd->q4_x100 >= 0) ? (cmd->q4_x100 + 50) / 100 : (cmd->q4_x100 - 50) / 100;
  int q5 = (cmd->q5_x100 >= 0) ? (cmd->q5_x100 + 50) / 100 : (cmd->q5_x100 - 50) / 100;
  int q6 = (cmd->q6_x100 >= 0) ? (cmd->q6_x100 + 50) / 100 : (cmd->q6_x100 - 50) / 100;

  snprintf(arm_command, ESP32_MAX_ARM_CMD_LEN + 1u,
           "%c:%d,%d,%d,%d,%d,%d", prefix, q1, q2, q3, q4, q5, q6);
  strncpy(esp32_data.ArmCommand, arm_command, sizeof(esp32_data.ArmCommand) - 1u);
  esp32_data.ArmCommand[sizeof(esp32_data.ArmCommand) - 1u] = '\0';
  *has_new_command = true;
  esp32_data.HasNewArmCommand = true;
}

/* --------------------------------------------------------------------------
 * CMD 0x01 — Sensor report (ESP32 → AGV main)
 * Payload: 8 bytes, all multi-byte fields are Little-Endian
 * -------------------------------------------------------------------------- */
static void ESP32_ProcessSensorReport(const uint8_t *payload, uint16_t payload_len) {
  if (payload_len != 8u) {
    dbg_rx_bad_len++;
    return;
  }

  int16_t  yaw_x100    = AGV_ProtoV2_ReadS16LE(&payload[0]);
  uint16_t obstacle_mm = AGV_ProtoV2_ReadU16LE(&payload[2]);
  uint16_t target_node = AGV_ProtoV2_ReadU16LE(&payload[4]);
  uint8_t  h_cmd       = payload[6];
  uint8_t  flags       = payload[7];

  if (flags & AGV_PROTO_V2_SENSOR_FLAG_IMU_VALID) {
    esp32_data.Yaw = (float)yaw_x100 / 100.0f;
  } else {
    esp32_data.Yaw = 65535.0f;
  }

  if (flags & AGV_PROTO_V2_SENSOR_FLAG_VL53_VALID) {
    esp32_data.ObstacleDistance = obstacle_mm;
  } else {
    esp32_data.ObstacleDistance = 0xFFFFu;
  }

  if (flags & AGV_PROTO_V2_SENSOR_FLAG_NEW_TARGET) {
    esp32_data.TargetNode   = (uint8_t)target_node;
    esp32_data.H_Command    = h_cmd;
    esp32_data.HasNewCommand = true;
  }

  esp32_data.LastUpdateTick = HAL_GetTick();
  esp32_data.IsConnected    = true;
  dbg_rx_success++;
}

/* --------------------------------------------------------------------------
 * CMD 0x20 — Arm joint command (PC → AGV main → Arm slave)
 *
 * V2.1 changes:
 *   - arm_id field REMOVED from payload; DEST header is the sole identifier
 *   - max_delta_x100 field ADDED at end of payload
 *   - Payload size: 22 bytes (was 18)
 *   - All int16/uint16 read as Little-Endian
 *
 * At this node (AGV main) we only bridge to legacy text — the delta guard
 * runs on the Arm slave. We store DEST so the bridge knows prefix (R/L).
 * -------------------------------------------------------------------------- */
static void ESP32_ProcessArmJointCommand(uint8_t dest,
                                         const uint8_t *payload,
                                         uint16_t payload_len) {
  dbg_arm_seen++;
  dbg_arm_last_dest = dest;
  dbg_arm_last_payload_len = payload_len;

  if (payload_len != 22u) {
    dbg_arm_bad_len++;
    dbg_rx_bad_len++;
    return;
  }

  AGV_ProtoV2_ArmJointCommand_t cmd;
  cmd.motion_mode    = payload[0];
  cmd.arm_flags      = payload[1];
  dbg_arm_last_motion_mode = cmd.motion_mode;
  dbg_arm_last_flags       = cmd.arm_flags;
  cmd.q1_x100        = AGV_ProtoV2_ReadS16LE(&payload[2]);
  cmd.q2_x100        = AGV_ProtoV2_ReadS16LE(&payload[4]);
  cmd.q3_x100        = AGV_ProtoV2_ReadS16LE(&payload[6]);
  cmd.q4_x100        = AGV_ProtoV2_ReadS16LE(&payload[8]);
  cmd.q5_x100        = AGV_ProtoV2_ReadS16LE(&payload[10]);
  cmd.q6_x100        = AGV_ProtoV2_ReadS16LE(&payload[12]);
  cmd.move_time_ms   = AGV_ProtoV2_ReadU16LE(&payload[14]);
  cmd.max_delta_x100 = AGV_ProtoV2_ReadU16LE(&payload[16]);
  /* bytes 18-21: reserved/padding in packed struct — ignored */

  if (cmd.motion_mode != AGV_PROTO_V2_MOTION_ABSOLUTE &&
      cmd.motion_mode != AGV_PROTO_V2_MOTION_HOME     &&
      cmd.motion_mode != AGV_PROTO_V2_MOTION_ESTOP) {
    dbg_arm_bad_mode++;
    dbg_rx_bad_len++;
    return;
  }

  ESP32_FormatLegacyArmCommand(dest, &cmd);
  dbg_arm_ok++;
  dbg_rx_success++;
}

/* --------------------------------------------------------------------------
 * Frame dispatcher — called once a complete, CRC-verified frame is buffered.
 *
 * V2.1 frame byte map:
 *   [0]=SOF1 [1]=SOF2 [2]=DEST [3]=SRC [4]=LEN_L [5]=LEN_H [6]=CMD [7]=SEQ
 *   [8..8+N-1]=PAYLOAD  [8+N]=CRC_L  [8+N+1]=CRC_H
 * -------------------------------------------------------------------------- */
static void ESP32_ProcessFrame(const uint8_t *frame, uint16_t frame_len) {
  if (frame_len < AGV_PROTO_V2_FRAME_OVERHEAD ||
      frame[AGV_PROTO_V2_OFF_SOF1] != AGV_PROTO_V2_SOF1 ||
      frame[AGV_PROTO_V2_OFF_SOF2] != AGV_PROTO_V2_SOF2) {
    dbg_rx_sync_lost++;
    return;
  }

  /* LEN is at offset 4:5, Little-Endian */
  uint16_t payload_len = AGV_ProtoV2_ReadU16LE(&frame[AGV_PROTO_V2_OFF_LEN_L]);
  uint8_t dest = frame[AGV_PROTO_V2_OFF_DEST];
  uint8_t src  = frame[AGV_PROTO_V2_OFF_SRC];
  uint8_t cmd  = frame[AGV_PROTO_V2_OFF_CMD];
  uint8_t seq  = frame[AGV_PROTO_V2_OFF_SEQ];

  dbg_rx_last_payload_len = payload_len;
  dbg_rx_last_frame_len   = frame_len;
  dbg_rx_last_dest = dest;
  dbg_rx_last_src  = src;
  dbg_rx_last_cmd  = cmd;
  dbg_rx_last_seq  = seq;

  if ((uint16_t)(payload_len + AGV_PROTO_V2_FRAME_OVERHEAD) != frame_len) {
    dbg_rx_bad_len++;
    return;
  }

  /* This node (AGV main) accepts frames destined for itself, broadcast, or arm slaves (to forward them) */
  if (dest != AGV_PROTO_V2_ADDR_MAIN &&
      dest != AGV_PROTO_V2_ADDR_BROADCAST &&
      dest != AGV_PROTO_V2_ADDR_ARM_LEFT &&
      dest != AGV_PROTO_V2_ADDR_ARM_RIGHT) {
    /* In a future phase: forward to RS485 arm slaves here */
    dbg_rx_bad_addr++;
    return;
  }

  /* CRC covers: DEST, SRC, LEN_L, LEN_H, CMD, SEQ, PAYLOAD
   * = bytes [2 .. 2 + 4 + 2 + payload_len - 1] = 6 + payload_len bytes */
  uint16_t rx_crc   = AGV_ProtoV2_ReadU16LE(&frame[8u + payload_len]);
  uint16_t calc_crc = AGV_ProtoV2_Crc16(&frame[2], (uint16_t)(6u + payload_len));
  dbg_rx_last_crc_rx   = rx_crc;
  dbg_rx_last_crc_calc = calc_crc;
  if (rx_crc != calc_crc) {
    dbg_rx_bad_cs++;
    return;
  }

  const uint8_t *payload = &frame[AGV_PROTO_V2_OFF_PAYLOAD];

  switch (cmd) {
  case AGV_PROTO_V2_CMD_SENSOR_REPORT:
    ESP32_ProcessSensorReport(payload, payload_len);
    break;

  case AGV_PROTO_V2_CMD_ARM_JOINT_COMMAND:
    /* Pass DEST so the legacy bridge knows R/L prefix */
    ESP32_ProcessArmJointCommand(dest, payload, payload_len);
    break;

  default:
    dbg_rx_unsupported_cmd++;
    dbg_rx_sync_lost++;
    break;
  }
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

void ESP32_Init(UART_HandleTypeDef *huart) {
  esp32_huart = huart;
  memset(&esp32_data, 0, sizeof(esp32_data));
  esp32_data.ObstacleDistance = 0xFFFFu;
  ESP32_ResetParser();

  HAL_UART_AbortReceive(esp32_huart);
  __HAL_UART_CLEAR_FLAG(esp32_huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                         UART_CLEAR_FEF | UART_CLEAR_PEF);
  HAL_UARTEx_ReceiveToIdle_DMA(esp32_huart, esp32_rx_buffer, sizeof(esp32_rx_buffer));
}

/* --------------------------------------------------------------------------
 * Build and transmit a CMD 0x11 Sync Request (AGV main → ESP32)
 * V2.1 header: [SOF1][SOF2][DEST][SRC][LEN_L][LEN_H][CMD][SEQ][PAYLOAD][CRC_L][CRC_H]
 * -------------------------------------------------------------------------- */
void ESP32_RequestData(uint16_t current_node, uint8_t is_arrived) {
  if (esp32_huart == NULL) return;

  const uint16_t payload_len = 4u;  /* SyncRequest: current_node(2) + is_arrived(1) + reserved(1) */
  uint8_t tx_frame[AGV_PROTO_V2_FRAME_OVERHEAD + 4u];

  tx_frame[AGV_PROTO_V2_OFF_SOF1]  = AGV_PROTO_V2_SOF1;
  tx_frame[AGV_PROTO_V2_OFF_SOF2]  = AGV_PROTO_V2_SOF2;
  tx_frame[AGV_PROTO_V2_OFF_DEST]  = AGV_PROTO_V2_ADDR_ESP32;
  tx_frame[AGV_PROTO_V2_OFF_SRC]   = AGV_PROTO_V2_ADDR_MAIN;
  AGV_ProtoV2_WriteU16LE(&tx_frame[AGV_PROTO_V2_OFF_LEN_L], payload_len);  /* [4][5] */
  tx_frame[AGV_PROTO_V2_OFF_CMD]   = AGV_PROTO_V2_CMD_SYNC_REQUEST;
  tx_frame[AGV_PROTO_V2_OFF_SEQ]   = esp32_seq_counter++;

  /* Payload @ offset 8 */
  AGV_ProtoV2_WriteU16LE(&tx_frame[8], current_node);
  tx_frame[10] = is_arrived;
  tx_frame[11] = 0u;   /* reserved */

  /* CRC covers bytes [2..11]: 6 header bytes (DEST..SEQ) + 4 payload bytes */
  uint16_t crc = AGV_ProtoV2_Crc16(&tx_frame[2], (uint16_t)(6u + payload_len));
  AGV_ProtoV2_WriteU16LE(&tx_frame[12], crc);

  HAL_StatusTypeDef status = HAL_UART_Transmit(esp32_huart, tx_frame, sizeof(tx_frame), 20);
  if (status != HAL_OK) {
    esp32_huart->gState = HAL_UART_STATE_READY;
    __HAL_UNLOCK(esp32_huart);
  }
}

/* --------------------------------------------------------------------------
 * Byte-by-byte state machine parser (called from DMA idle callback)
 *
 * V2.1 State Machine:
 *   0 = WAIT_SOF1
 *   1 = WAIT_SOF2
 *   2 = ACCUMULATE (DEST, SRC, LEN_L, LEN_H, CMD, SEQ, PAYLOAD, CRC)
 *
 * Key change: LEN is at offset [4:5] so esp32_expected_len is known after
 * receiving 6 bytes (index == 6), allowing DMA to pre-size the remainder.
 * -------------------------------------------------------------------------- */
void ESP32_ParseResponse(uint16_t length) {
  if (length > sizeof(esp32_rx_buffer)) {
    length = sizeof(esp32_rx_buffer);
  }

  for (uint16_t i = 0; i < length; i++) {
    uint8_t b = esp32_rx_buffer[i];

    switch (esp32_frame_state) {

    /* ---- State 0: waiting for SOF1 ---- */
    case 0:
      if (b == AGV_PROTO_V2_SOF1) {
        esp32_frame_buffer[0] = b;
        esp32_frame_index     = 1;
        esp32_frame_state     = 1;
      }
      break;

    /* ---- State 1: waiting for SOF2 ---- */
    case 1:
      if (b == AGV_PROTO_V2_SOF2) {
        esp32_frame_buffer[1] = b;
        esp32_frame_index     = 2;
        esp32_frame_state     = 2;
      } else if (b == AGV_PROTO_V2_SOF1) {
        /* consecutive SOF1 — stay in state 1 */
        esp32_frame_buffer[0] = b;
        esp32_frame_index     = 1;
      } else {
        ESP32_ResetParser();
      }
      break;

    /* ---- State 2: accumulate remaining bytes ---- */
    case 2:
      if (esp32_frame_index >= sizeof(esp32_frame_buffer)) {
        dbg_rx_bad_len++;
        ESP32_ResetParser();
        break;
      }

      esp32_frame_buffer[esp32_frame_index++] = b;

      /* After [SOF1][SOF2][DEST][SRC][LEN_L][LEN_H] we have index == 6.
       * LEN is at offsets 4:5 — compute total expected frame length now.  */
      if (esp32_frame_index == 6u) {
        uint16_t payload_len = AGV_ProtoV2_ReadU16LE(&esp32_frame_buffer[4]);
        if (payload_len > AGV_PROTO_V2_MAX_PAYLOAD_LEN) {
          dbg_rx_bad_len++;
          ESP32_ResetParser();
          break;
        }
        /* total = 10 overhead + payload_len */
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

  /* Connectivity watchdog */
  if (HAL_GetTick() - esp32_data.LastUpdateTick > 1000u) {
    esp32_data.IsConnected = false;
  }
}

/* --------------------------------------------------------------------------
 * Thread-safe data snapshot
 * -------------------------------------------------------------------------- */
ESP32_SensorData_t ESP32_GetSafeData(void) {
  ESP32_SensorData_t copy;
  __disable_irq();
  copy = esp32_data;
  __enable_irq();
  return copy;
}
