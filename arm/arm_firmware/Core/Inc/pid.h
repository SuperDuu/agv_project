#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float setpoint;
    float integral;
    float prev_error;
    
    float out_min;
    float out_max;
    
    float dt; // Sample time in seconds
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max, float dt);
float PID_Compute(PID_t *pid, float setpoint, float feedback);
void PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
