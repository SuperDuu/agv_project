#ifndef AGV_LINK_PROTOCOL_H
#define AGV_LINK_PROTOCOL_H

/* ==========================================================================
 * AGV/ARM Link Protocol V2.1
 * Changes from V2:
 *  - Full Little-Endian (all multi-byte fields)
 *  - LEN field moved before CMD/SEQ for DMA pre-allocation
 *  - arm_id removed from CMD 0x20 / 0x21 (DEST header is the sole identifier)
 *  - max_delta_x100 added to CMD 0x20 for Slave-side Δθ safety enforcement
 *  - CMD 0x50 split: 0x50 = AGV status, 0x51 = Arm status
 *  - Error code 0x06 = joint delta exceeded
 * ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* --- SOF bytes ------------------------------------------------------------ */
#define AGV_PROTO_V2_SOF1             0xAAu
#define AGV_PROTO_V2_SOF2             0x55u

/* --- Node addresses ------------------------------------------------------- */
#define AGV_PROTO_V2_ADDR_MAIN        0x01u
#define AGV_PROTO_V2_ADDR_ARM_LEFT    0x02u
#define AGV_PROTO_V2_ADDR_ARM_RIGHT   0x03u
#define AGV_PROTO_V2_ADDR_ESP32       0x10u
#define AGV_PROTO_V2_ADDR_PC_APP      0x20u
#define AGV_PROTO_V2_ADDR_BROADCAST   0x7Fu

/* --- Command IDs ---------------------------------------------------------- */
#define AGV_PROTO_V2_CMD_SENSOR_REPORT        0x01u
#define AGV_PROTO_V2_CMD_AGV_COMMAND          0x10u
#define AGV_PROTO_V2_CMD_SYNC_REQUEST         0x11u
#define AGV_PROTO_V2_CMD_ARM_JOINT_COMMAND    0x20u
#define AGV_PROTO_V2_CMD_ARM_GRIPPER_COMMAND  0x21u
#define AGV_PROTO_V2_CMD_ACK                  0x30u
#define AGV_PROTO_V2_CMD_NACK                 0x31u
#define AGV_PROTO_V2_CMD_HEARTBEAT            0x40u
#define AGV_PROTO_V2_CMD_AGV_STATUS_REPORT    0x50u  /* AGV main state only  */
#define AGV_PROTO_V2_CMD_ARM_STATUS_REPORT    0x51u  /* Arm slave state only */

/* --- Motion modes --------------------------------------------------------- */
#define AGV_PROTO_V2_MOTION_HOLD      0u
#define AGV_PROTO_V2_MOTION_ABSOLUTE  1u
#define AGV_PROTO_V2_MOTION_RELATIVE  2u
#define AGV_PROTO_V2_MOTION_HOME      3u
#define AGV_PROTO_V2_MOTION_ESTOP     4u

/* --- Gripper actions ------------------------------------------------------ */
#define AGV_PROTO_V2_GRIP_RELEASE     0u
#define AGV_PROTO_V2_GRIP_CLOSE       1u
#define AGV_PROTO_V2_GRIP_TOGGLE      2u

/* --- Error codes (NACK) --------------------------------------------------- */
#define AGV_PROTO_V2_ERR_BAD_LENGTH           1u
#define AGV_PROTO_V2_ERR_BAD_CRC             2u
#define AGV_PROTO_V2_ERR_BAD_ADDRESS         3u
#define AGV_PROTO_V2_ERR_UNSUPPORTED_CMD     4u
#define AGV_PROTO_V2_ERR_INVALID_PAYLOAD     5u
#define AGV_PROTO_V2_ERR_JOINT_DELTA_EXCEEDED 6u  /* Δθ > max_delta_x100 */
#define AGV_PROTO_V2_ERR_BUSY                7u

/* --- Sensor flags --------------------------------------------------------- */
#define AGV_PROTO_V2_SENSOR_FLAG_IMU_VALID  (1u << 0)
#define AGV_PROTO_V2_SENSOR_FLAG_VL53_VALID (1u << 1)
#define AGV_PROTO_V2_SENSOR_FLAG_NEW_TARGET (1u << 2)

/* --- Arm flags ------------------------------------------------------------ */
#define AGV_PROTO_V2_ARM_FLAG_VALID          (1u << 0)
#define AGV_PROTO_V2_ARM_FLAG_GRIP_INCLUDED  (1u << 1)
#define AGV_PROTO_V2_ARM_FLAG_SYNC_WITH_AGV  (1u << 2)

/* --- Frame layout (V2.1) --------------------------------------------------
 * [SOF1][SOF2][DEST][SRC][LEN_L][LEN_H][CMD][SEQ][PAYLOAD...][CRC_L][CRC_H]
 *   0     1     2    3     4      5      6    7     8..8+N-1   8+N   8+N+1
 * FRAME_OVERHEAD = 10 (2 SOF + 1 DEST + 1 SRC + 2 LEN + 1 CMD + 1 SEQ + 2 CRC)
 * LEN = payload_len (uint16, Little-Endian)
 * CRC covers: DEST, SRC, LEN_L, LEN_H, CMD, SEQ, PAYLOAD
 * ----------------------------------------------------------------------- */
#define AGV_PROTO_V2_FRAME_OVERHEAD   10u
#define AGV_PROTO_V2_MAX_PAYLOAD_LEN  64u
#define AGV_PROTO_V2_MAX_FRAME_LEN    (AGV_PROTO_V2_FRAME_OVERHEAD + AGV_PROTO_V2_MAX_PAYLOAD_LEN)

/* Byte offsets within a raw frame buffer */
#define AGV_PROTO_V2_OFF_SOF1   0u
#define AGV_PROTO_V2_OFF_SOF2   1u
#define AGV_PROTO_V2_OFF_DEST   2u
#define AGV_PROTO_V2_OFF_SRC    3u
#define AGV_PROTO_V2_OFF_LEN_L  4u   /* LEN low byte  (LE) */
#define AGV_PROTO_V2_OFF_LEN_H  5u   /* LEN high byte (LE) */
#define AGV_PROTO_V2_OFF_CMD    6u
#define AGV_PROTO_V2_OFF_SEQ    7u
#define AGV_PROTO_V2_OFF_PAYLOAD 8u

/* --- Payload structs (packed for wire layout) ----------------------------- */

typedef struct __attribute__((packed)) {
  int16_t  yaw_x100;       /* IMU yaw (deg * 100), LE */
  uint16_t obstacle_mm;    /* VL53 distance, LE       */
  uint16_t target_node;    /* destination node, LE    */
  uint8_t  h_cmd;
  uint8_t  sensor_flags;
} AGV_ProtoV2_SensorReport_t;   /* 8 bytes */

typedef struct __attribute__((packed)) {
  uint16_t target_node;    /* LE */
  uint8_t  move_mode;
  uint8_t  command_flags;
} AGV_ProtoV2_AGVCommand_t;     /* 4 bytes */

typedef struct __attribute__((packed)) {
  uint16_t current_node;   /* LE */
  uint8_t  is_arrived;
  uint8_t  reserved;
} AGV_ProtoV2_SyncRequest_t;    /* 4 bytes */

/* CMD 0x20 — arm_id REMOVED; use frame DEST to identify left/right arm */
typedef struct __attribute__((packed)) {
  uint8_t  motion_mode;
  uint8_t  arm_flags;
  int16_t  q1_x100;        /* LE */
  int16_t  q2_x100;
  int16_t  q3_x100;
  int16_t  q4_x100;
  int16_t  q5_x100;
  int16_t  q6_x100;
  uint16_t move_time_ms;   /* LE */
  uint16_t max_delta_x100; /* LE — Slave drops frame if Δθ > this; 0 = no limit */
} AGV_ProtoV2_ArmJointCommand_t; /* 22 bytes */

/* CMD 0x21 — arm_id REMOVED */
typedef struct __attribute__((packed)) {
  uint8_t  grip_action;
  uint8_t  reserved[3];
} AGV_ProtoV2_ArmGripperCommand_t; /* 4 bytes */

typedef struct __attribute__((packed)) {
  uint8_t  acked_cmd;
  uint8_t  acked_seq;
  uint8_t  status;         /* 0=ok, 1=busy, 2=queued */
  uint8_t  reserved;
} AGV_ProtoV2_Ack_t;       /* 4 bytes */

typedef struct __attribute__((packed)) {
  uint8_t  nack_cmd;
  uint8_t  nack_seq;
  uint8_t  error_code;
  uint8_t  reserved;
} AGV_ProtoV2_Nack_t;      /* 4 bytes */

typedef struct __attribute__((packed)) {
  uint32_t uptime_ms;      /* LE */
  uint8_t  state;
  uint8_t  reserved[3];
} AGV_ProtoV2_Heartbeat_t; /* 8 bytes */

/* CMD 0x50 — AGV main status only */
typedef struct __attribute__((packed)) {
  uint8_t  node_state;     /* 0=idle, 1=running, 2=error, 3=estop */
  uint8_t  error_code;
  uint16_t current_node;   /* LE */
  uint16_t target_node;    /* LE */
  uint8_t  agv_flags;
  uint8_t  reserved;
} AGV_ProtoV2_AGVStatusReport_t; /* 8 bytes */

/* CMD 0x51 — Arm slave status; SRC = 0x02 or 0x03 identifies which arm */
typedef struct __attribute__((packed)) {
  uint8_t  arm_state;      /* 0=idle, 1=moving, 2=error, 3=estop */
  uint8_t  error_code;
  int16_t  q1_x100;        /* LE — current joint feedback */
  int16_t  q2_x100;
  int16_t  q3_x100;
  int16_t  q4_x100;
  int16_t  q5_x100;
  int16_t  q6_x100;
  uint8_t  gripper_state;  /* 0=open, 1=closed, 2=moving */
  uint8_t  reserved;
} AGV_ProtoV2_ArmStatusReport_t; /* 16 bytes */

/* --- Generic frame view --------------------------------------------------- */
typedef struct {
  uint8_t        dest;
  uint8_t        src;
  uint8_t        cmd;
  uint8_t        seq;
  uint16_t       payload_len;
  const uint8_t *payload;
} AGV_ProtoV2_FrameView_t;

/* --- Utility: angle conversion ------------------------------------------- */
static inline int16_t AGV_ProtoV2_DegToX100(float angle_deg) {
  if (angle_deg >= 0.0f) return (int16_t)(angle_deg * 100.0f + 0.5f);
  return (int16_t)(angle_deg * 100.0f - 0.5f);
}

static inline float AGV_ProtoV2_X100ToDeg(int16_t angle_x100) {
  return (float)angle_x100 / 100.0f;
}

/* --- Utility: Little-Endian read/write ------------------------------------ */
static inline void AGV_ProtoV2_WriteU16LE(uint8_t *dst, uint16_t value) {
  dst[0] = (uint8_t)(value & 0xFFu);
  dst[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static inline void AGV_ProtoV2_WriteS16LE(uint8_t *dst, int16_t value) {
  AGV_ProtoV2_WriteU16LE(dst, (uint16_t)value);
}

static inline uint16_t AGV_ProtoV2_ReadU16LE(const uint8_t *src) {
  return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

static inline int16_t AGV_ProtoV2_ReadS16LE(const uint8_t *src) {
  return (int16_t)AGV_ProtoV2_ReadU16LE(src);
}

/* Compatibility aliases — point old BE names to LE implementations so
 * call sites that haven't been migrated yet still compile.             */
static inline void     AGV_ProtoV2_WriteU16BE(uint8_t *d, uint16_t v) { AGV_ProtoV2_WriteU16LE(d, v); }
static inline void     AGV_ProtoV2_WriteS16BE(uint8_t *d, int16_t  v) { AGV_ProtoV2_WriteS16LE(d, v); }
static inline uint16_t AGV_ProtoV2_ReadU16BE(const uint8_t *s)        { return AGV_ProtoV2_ReadU16LE(s); }
static inline int16_t  AGV_ProtoV2_ReadS16BE(const uint8_t *s)        { return AGV_ProtoV2_ReadS16LE(s); }

#ifdef __cplusplus
}
#endif

#endif /* AGV_LINK_PROTOCOL_H */
