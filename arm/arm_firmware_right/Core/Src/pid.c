#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max, float dt) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->dt = (dt > 0.0f) ? dt : 0.001f; // Avoid division by zero in derivative term
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid) {
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Compute(PID_t *pid, float setpoint, float feedback) {
    pid->setpoint = setpoint;
    float error = setpoint - feedback;
    
    // Proportional term
    float P_out = pid->Kp * error;
    
    // Integral term (pre-accumulation)
    pid->integral += error * pid->dt;
    float I_out = pid->Ki * pid->integral;
    
    // Derivative term
    float derivative = (error - pid->prev_error) / pid->dt;
    float D_out = pid->Kd * derivative;
    
    // Total raw output
    float output = P_out + I_out + D_out;
    
    // Saturation and Anti-windup
    if (output > pid->out_max) {
        output = pid->out_max;
        // Clamp integral accumulator if the error has the same sign as output limit
        if (error > 0.0f) {
            pid->integral -= error * pid->dt;
        }
    } else if (output < pid->out_min) {
        output = pid->out_min;
        // Clamp integral accumulator if the error has the same sign as output limit
        if (error < 0.0f) {
            pid->integral -= error * pid->dt;
        }
    }
    
    // Save current error for next loop
    pid->prev_error = error;
    
    return output;
}
