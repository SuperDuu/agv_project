#ifndef AGV_LINK_PROTOCOL_H
#define AGV_LINK_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define AGV_PROTO_V2_SOF1 0xAA
#define AGV_PROTO_V2_SOF2 0x55

#define AGV_PROTO_V2_ADDR_MAIN 0x01
#define AGV_PROTO_V2_ADDR_ARM_LEFT 0x02
#define AGV_PROTO_V2_ADDR_ARM_RIGHT 0x03
#define AGV_PROTO_V2_ADDR_ESP32 0x10
#define AGV_PROTO_V2_ADDR_PC_APP 0x20
#define AGV_PROTO_V2_ADDR_BROADCAST 0x7F

#define AGV_PROTO_V2_CMD_SENSOR_REPORT 0x01
#define AGV_PROTO_V2_CMD_AGV_COMMAND 0x10
#define AGV_PROTO_V2_CMD_SYNC_REQUEST 0x11
#define AGV_PROTO_V2_CMD_ARM_JOINT_COMMAND 0x20
#define AGV_PROTO_V2_CMD_ARM_GRIPPER_COMMAND 0x21
#define AGV_PROTO_V2_CMD_ACK 0x30
#define AGV_PROTO_V2_CMD_NACK 0x31
#define AGV_PROTO_V2_CMD_HEARTBEAT 0x40
#define AGV_PROTO_V2_CMD_STATUS_REPORT 0x50

#define AGV_PROTO_V2_ARM_LEFT_ID 0
#define AGV_PROTO_V2_ARM_RIGHT_ID 1

#define AGV_PROTO_V2_MOTION_HOLD 0
#define AGV_PROTO_V2_MOTION_ABSOLUTE 1
#define AGV_PROTO_V2_MOTION_RELATIVE 2
#define AGV_PROTO_V2_MOTION_HOME 3
#define AGV_PROTO_V2_MOTION_ESTOP 4

#define AGV_PROTO_V2_GRIP_RELEASE 0
#define AGV_PROTO_V2_GRIP_CLOSE 1
#define AGV_PROTO_V2_GRIP_TOGGLE 2

#define AGV_PROTO_V2_ERR_BAD_LENGTH 1
#define AGV_PROTO_V2_ERR_BAD_CRC 2
#define AGV_PROTO_V2_ERR_BAD_ADDRESS 3
#define AGV_PROTO_V2_ERR_UNSUPPORTED_CMD 4
#define AGV_PROTO_V2_ERR_INVALID_PAYLOAD 5
#define AGV_PROTO_V2_ERR_OUT_OF_RANGE 6
#define AGV_PROTO_V2_ERR_BUSY 7

#define AGV_PROTO_V2_SENSOR_FLAG_IMU_VALID (1u << 0)
#define AGV_PROTO_V2_SENSOR_FLAG_VL53_VALID (1u << 1)
#define AGV_PROTO_V2_SENSOR_FLAG_NEW_TARGET (1u << 2)

#define AGV_PROTO_V2_ARM_FLAG_VALID (1u << 0)
#define AGV_PROTO_V2_ARM_FLAG_GRIP_INCLUDED (1u << 1)
#define AGV_PROTO_V2_ARM_FLAG_SYNC_WITH_AGV (1u << 2)

#define AGV_PROTO_V2_FRAME_OVERHEAD 10u
#define AGV_PROTO_V2_MAX_PAYLOAD_LEN 64u
#define AGV_PROTO_V2_MAX_FRAME_LEN \
  (AGV_PROTO_V2_FRAME_OVERHEAD + AGV_PROTO_V2_MAX_PAYLOAD_LEN)

typedef struct {
  uint8_t dest;
  uint8_t src;
  uint8_t cmd;
  uint8_t seq;
  uint16_t payload_len;
  const uint8_t *payload;
} AGV_ProtoV2_FrameView_t;

typedef struct {
  int16_t yaw_x100;
  uint16_t obstacle_mm;
  uint16_t target_node;
  uint8_t h_cmd;
  uint8_t sensor_flags;
} AGV_ProtoV2_SensorReport_t;

typedef struct {
  uint16_t target_node;
  uint8_t move_mode;
  uint8_t command_flags;
} AGV_ProtoV2_AGVCommand_t;

typedef struct {
  uint16_t current_node;
  uint8_t is_arrived;
  uint8_t reserved;
} AGV_ProtoV2_SyncRequest_t;

typedef struct {
  uint8_t arm_id;
  uint8_t motion_mode;
  int16_t q1_x100;
  int16_t q2_x100;
  int16_t q3_x100;
  int16_t q4_x100;
  int16_t q5_x100;
  int16_t q6_x100;
  uint16_t move_time_ms;
  uint8_t arm_flags;
  uint8_t reserved;
} AGV_ProtoV2_ArmJointCommand_t;

typedef struct {
  uint8_t arm_id;
  uint8_t grip_action;
  uint16_t grip_force;
  uint16_t reserved;
} AGV_ProtoV2_ArmGripperCommand_t;

typedef struct {
  uint8_t acked_cmd;
  uint8_t acked_seq;
  uint8_t status;
  uint8_t reserved;
} AGV_ProtoV2_Ack_t;

typedef struct {
  uint8_t nack_cmd;
  uint8_t nack_seq;
  uint8_t error_code;
  uint8_t reserved;
} AGV_ProtoV2_Nack_t;

typedef struct {
  uint32_t uptime_ms;
  uint8_t state;
  uint8_t reserved[3];
} AGV_ProtoV2_Heartbeat_t;

typedef struct {
  uint8_t node_state;
  uint8_t error_code;
  uint16_t current_node;
  int16_t q1_x100;
  int16_t q2_x100;
  int16_t q3_x100;
  int16_t q4_x100;
  int16_t q5_x100;
  int16_t q6_x100;
} AGV_ProtoV2_StatusReport_t;

static inline int16_t AGV_ProtoV2_DegToX100(float angle_deg) {
  if (angle_deg >= 0.0f) {
    return (int16_t)(angle_deg * 100.0f + 0.5f);
  }
  return (int16_t)(angle_deg * 100.0f - 0.5f);
}

static inline float AGV_ProtoV2_X100ToDeg(int16_t angle_x100) {
  return (float)angle_x100 / 100.0f;
}

static inline void AGV_ProtoV2_WriteU16BE(uint8_t *dst, uint16_t value) {
  dst[0] = (uint8_t)((value >> 8) & 0xFFu);
  dst[1] = (uint8_t)(value & 0xFFu);
}

static inline void AGV_ProtoV2_WriteS16BE(uint8_t *dst, int16_t value) {
  AGV_ProtoV2_WriteU16BE(dst, (uint16_t)value);
}

static inline uint16_t AGV_ProtoV2_ReadU16BE(const uint8_t *src) {
  return (uint16_t)(((uint16_t)src[0] << 8) | (uint16_t)src[1]);
}

static inline int16_t AGV_ProtoV2_ReadS16BE(const uint8_t *src) {
  return (int16_t)AGV_ProtoV2_ReadU16BE(src);
}

#ifdef __cplusplus
}
#endif

#endif
