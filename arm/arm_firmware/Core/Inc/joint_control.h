#ifndef __JOINT_CONTROL_H
#define __JOINT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"
#include "encoder.h"
#include "servo.h"

#define NUM_JOINTS 5

typedef enum {
    JOINT_MODE_IDLE = 0,
    JOINT_MODE_VEL_TUNING = 1,
    JOINT_MODE_CASCADE = 2
} JointMode_t;

typedef struct {
    JointMode_t mode;
    uint8_t encoder_idx;
    uint8_t servo_idx;
    
    // Control variables
    float target_pos;      // Target position in ticks
    float target_vel;      // Target velocity in ticks/sec
    
    float current_pos;     // Current position in ticks
    float current_vel;     // Current velocity in ticks/sec
    float last_pos;        // Last position for velocity calculation
    
    PID_t pid_pos;         // Position PID (outer loop)
    PID_t pid_vel;         // Velocity PID (inner loop)
    
    float servo_command_angle; // Current servo command (0 to 270 degrees)
} JointController_t;

extern JointController_t joints[NUM_JOINTS];

void JointControl_Init(void);
void JointControl_Update(float dt);
void JointControl_SetMode(uint8_t joint_idx, JointMode_t mode);
void JointControl_SetTarget(uint8_t joint_idx, float target);
void JointControl_TunePID(uint8_t joint_idx, uint8_t is_pos_loop, float Kp, float Ki, float Kd);

#ifdef __cplusplus
}
#endif

#endif /* __JOINT_CONTROL_H */
