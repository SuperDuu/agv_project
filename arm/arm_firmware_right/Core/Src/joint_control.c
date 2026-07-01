#include "joint_control.h"

// Global array of joint controllers
JointController_t joints[NUM_JOINTS];

void JointControl_Init(void) {
    // Default sample time for PID calculations (e.g. 10ms)
    float default_dt = 0.010f; 
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        JointController_t *joint = &joints[i];
        
        joint->mode = JOINT_MODE_IDLE;
        joint->encoder_idx = i;
        joint->servo_idx = i; // Maps joint i to servo i
        
        joint->target_pos = 0.0f;
        joint->target_vel = 0.0f;
        joint->current_pos = 0.0f;
        joint->current_vel = 0.0f;
        joint->last_pos = 0.0f;
        
        // Initial neutral servo angle command (center of range)
        if (i == 0) {
            joint->servo_command_angle = 96.43f; // 96.43 deg joint = 135 deg servo
        } else if (i == 1) {
            joint->servo_command_angle = 10.0f;  // 10 deg joint = 14 deg servo
        } else if (i == 2) {
            joint->servo_command_angle = 192.86f; // 192.86 deg joint = 270 deg servo
        } else if (i == 3 || i == 5) {
            joint->servo_command_angle = 90.0f;  // Middle of 180-degree servo
        } else if (i == 4) {
            joint->servo_command_angle = 60.0f;  // Middle of joint (90 deg servo)
        } else {
            joint->servo_command_angle = 135.0f; 
        }
        
        // Position PID (outer loop)
        // Setpoint: target position (ticks)
        // Feedback: current position (ticks)
        // Output: target velocity (ticks/sec)
        // Limits: e.g. -2000 to 2000 ticks/sec
        PID_Init(&joint->pid_pos, 0.5f, 0.0f, 0.0f, -2000.0f, 2000.0f, default_dt);
        
        // Velocity PID (inner loop)
        // Setpoint: target velocity (ticks/sec)
        // Feedback: current velocity (ticks/sec)
        // Output: correction rate for servo command (degrees/sec)
        // Limits: e.g. -100 to 100 degrees/sec
        PID_Init(&joint->pid_vel, 0.1f, 0.05f, 0.0f, -100.0f, 100.0f, default_dt);
    }
}

void JointControl_Update(float dt) {
    if (dt <= 0.0f) dt = 0.010f; // Prevent division by zero
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        JointController_t *joint = &joints[i];
        
        // 1. Read position feedback (ticks)
        joint->current_pos = (float)Encoder_Get_Ticks(joint->encoder_idx);
        
        // 2. Calculate current velocity (ticks/sec)
        joint->current_vel = (joint->current_pos - joint->last_pos) / dt;
        joint->last_pos = joint->current_pos;
        
        if (joint->mode == JOINT_MODE_IDLE) {
            continue; // Keep current servo command angle, do not calculate PID
        }
        
        // 3. Compute control outputs based on active mode
        float target_vel = 0.0f;
        
        if (joint->mode == JOINT_MODE_CASCADE) {
            // Position Loop (outer) -> Output is target velocity
            target_vel = PID_Compute(&joint->pid_pos, joint->target_pos, joint->current_pos);
            joint->target_vel = target_vel;
        } else if (joint->mode == JOINT_MODE_VEL_TUNING) {
            // Direct Velocity Tuning Mode
            target_vel = joint->target_vel;
        }
        
        // Velocity Loop (inner) -> Output is the change rate of servo angle (degrees/sec)
        float servo_rate = PID_Compute(&joint->pid_vel, target_vel, joint->current_vel);
        
        // Integrate the rate of change to get absolute command angle
        joint->servo_command_angle += servo_rate * dt;
        
        // Saturation clamping based on physical/gearbox limits
        float max_clamp;
        if (i <= 2) {
            max_clamp = 192.86f; // 5:7 gearbox on 270 deg servo
        } else if (i == 3 || i == 5) {
            max_clamp = 180.0f;  // 180 deg servo
        } else if (i == 4) {
            max_clamp = 120.0f;  // 2:3 gearbox on 180 deg servo (180 * 2 / 3)
        } else {
            max_clamp = 270.0f;  // 270 deg servo
        }
        if (joint->servo_command_angle > max_clamp) joint->servo_command_angle = max_clamp;
        if (joint->servo_command_angle < 0.0f) joint->servo_command_angle = 0.0f;
        
        // 4. Update Servo Position
        Set_Servo_Angle(joint->servo_idx, joint->servo_command_angle);
    }
}

void JointControl_SetMode(uint8_t joint_idx, JointMode_t mode) {
    if (joint_idx >= NUM_JOINTS) return;
    
    // If transitioning from IDLE, align command angle with neutral or current position to avoid jerking
    if (joints[joint_idx].mode == JOINT_MODE_IDLE && mode != JOINT_MODE_IDLE) {
        PID_Reset(&joints[joint_idx].pid_pos);
        PID_Reset(&joints[joint_idx].pid_vel);
    }
    
    joints[joint_idx].mode = mode;
}

void JointControl_SetTarget(uint8_t joint_idx, float target) {
    if (joint_idx >= NUM_JOINTS) return;
    
    if (joints[joint_idx].mode == JOINT_MODE_CASCADE) {
        joints[joint_idx].target_pos = target;
    } else if (joints[joint_idx].mode == JOINT_MODE_VEL_TUNING) {
        joints[joint_idx].target_vel = target;
    }
}

void JointControl_TunePID(uint8_t joint_idx, uint8_t is_pos_loop, float Kp, float Ki, float Kd) {
    if (joint_idx >= NUM_JOINTS) return;
    
    JointController_t *joint = &joints[joint_idx];
    if (is_pos_loop) {
        PID_Init(&joint->pid_pos, Kp, Ki, Kd, joint->pid_pos.out_min, joint->pid_pos.out_max, joint->pid_pos.dt);
    } else {
        PID_Init(&joint->pid_vel, Kp, Ki, Kd, joint->pid_vel.out_min, joint->pid_vel.out_max, joint->pid_vel.dt);
    }
}
