#ifndef ARM_PROTOCOL_H
#define ARM_PROTOCOL_H

/* ==========================================================================
 * arm_protocol.h  —  AGV/ARM Link Protocol V2.1 (Arm Slave subset)
 *
 * This header is a self-contained subset of the full agv_link_protocol.h,
 * tailored for the STM32F4 arm slave nodes (arm_firmware_right / left).
 *
 * Frame layout:
 *   [SOF1][SOF2][DEST][SRC][LEN_L][LEN_H][CMD][SEQ][PAYLOAD...][CRC_L][CRC_H]
 *
 * All multi-byte fields: Little-Endian.
 * LEN = number of PAYLOAD bytes (NOT including header or CRC).
 * CRC-16/CCITT-FALSE covers: DEST, SRC, LEN_L, LEN_H, CMD, SEQ, PAYLOAD.
 * ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* --- SOF ------------------------------------------------------------------ */
#define ARM_PROTO_SOF1            0xAAu
#define ARM_PROTO_SOF2            0x55u

/* --- This node's address (compile-time selection) ------------------------- */
#ifndef ARM_PROTO_MY_ADDR
#  define ARM_PROTO_MY_ADDR       0x02u   /* default: Left arm. Override in Makefile/IDE for Right (0x03) */
#endif
#define ARM_PROTO_ADDR_MAIN       0x01u
#define ARM_PROTO_ADDR_ARM_LEFT   0x02u
#define ARM_PROTO_ADDR_ARM_RIGHT  0x03u
#define ARM_PROTO_ADDR_BROADCAST  0x7Fu

/* --- Command IDs ---------------------------------------------------------- */
#define ARM_PROTO_CMD_ARM_JOINT   0x20u
#define ARM_PROTO_CMD_ARM_GRIPPER 0x21u
#define ARM_PROTO_CMD_ACK         0x30u
#define ARM_PROTO_CMD_NACK        0x31u
#define ARM_PROTO_CMD_HEARTBEAT   0x40u
#define ARM_PROTO_CMD_ARM_STATUS  0x51u   /* Arm slave → AGV main */

/* --- Motion modes --------------------------------------------------------- */
#define ARM_PROTO_MOTION_HOLD     0u
#define ARM_PROTO_MOTION_ABS      1u
#define ARM_PROTO_MOTION_REL      2u
#define ARM_PROTO_MOTION_HOME     3u
#define ARM_PROTO_MOTION_ESTOP    4u

/* --- NACK error codes ----------------------------------------------------- */
#define ARM_PROTO_ERR_BAD_LENGTH          1u
#define ARM_PROTO_ERR_BAD_CRC            2u
#define ARM_PROTO_ERR_BAD_ADDRESS        3u
#define ARM_PROTO_ERR_UNSUPPORTED_CMD    4u
#define ARM_PROTO_ERR_INVALID_PAYLOAD    5u
#define ARM_PROTO_ERR_JOINT_DELTA        6u   /* Δθ exceeded max_delta_x100 */
#define ARM_PROTO_ERR_BUSY               7u

/* --- Frame constants ------------------------------------------------------ */
#define ARM_PROTO_FRAME_OVERHEAD  10u          /* 2+1+1+2+1+1+2 */
#define ARM_PROTO_MAX_PAYLOAD     64u
#define ARM_PROTO_MAX_FRAME       (ARM_PROTO_FRAME_OVERHEAD + ARM_PROTO_MAX_PAYLOAD)

/* Payload sizes (compile-time constants for validation) */
#define ARM_PROTO_PAYLOAD_JOINT   22u    /* CMD 0x20 */
#define ARM_PROTO_PAYLOAD_GRIPPER  4u    /* CMD 0x21 */
#define ARM_PROTO_PAYLOAD_ACK      4u
#define ARM_PROTO_PAYLOAD_NACK     4u
#define ARM_PROTO_PAYLOAD_STATUS  16u    /* CMD 0x51 */

/* --- Byte offsets in raw frame ------------------------------------------- */
#define ARM_PROTO_OFF_SOF1    0u
#define ARM_PROTO_OFF_SOF2    1u
#define ARM_PROTO_OFF_DEST    2u
#define ARM_PROTO_OFF_SRC     3u
#define ARM_PROTO_OFF_LEN_L   4u
#define ARM_PROTO_OFF_LEN_H   5u
#define ARM_PROTO_OFF_CMD     6u
#define ARM_PROTO_OFF_SEQ     7u
#define ARM_PROTO_OFF_PAYLOAD 8u

/* --- Little-Endian helpers ------------------------------------------------ */
static inline uint16_t ARM_Proto_ReadU16LE(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline int16_t ARM_Proto_ReadS16LE(const uint8_t *p) {
  return (int16_t)ARM_Proto_ReadU16LE(p);
}
static inline void ARM_Proto_WriteU16LE(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

/* --- Angle conversion ----------------------------------------------------- */
static inline float ARM_Proto_X100ToDeg(int16_t x100) {
  return (float)x100 / 100.0f;
}
static inline int16_t ARM_Proto_DegToX100(float deg) {
  return (deg >= 0.0f) ? (int16_t)(deg * 100.0f + 0.5f)
                       : (int16_t)(deg * 100.0f - 0.5f);
}

/* --- CRC-16/CCITT-FALSE --------------------------------------------------- */
static inline uint16_t ARM_Proto_Crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFFu;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8u; b++) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/* --- Parsed joint command ------------------------------------------------- */
typedef struct {
  uint8_t  motion_mode;
  uint8_t  arm_flags;
  int16_t  q[6];            /* q[0]..q[5] in x100 fixed-point */
  uint16_t move_time_ms;
  uint16_t max_delta_x100;
} ARM_Proto_JointCmd_t;

/* --- Parsed gripper command ----------------------------------------------- */
typedef struct {
  uint8_t grip_action;      /* 0=release, 1=grip, 2=toggle */
} ARM_Proto_GripperCmd_t;

#ifdef __cplusplus
}
#endif

#endif /* ARM_PROTOCOL_H */
