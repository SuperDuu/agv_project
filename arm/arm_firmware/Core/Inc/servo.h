#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define MAX_SERVOS 10

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t min_pulse; // 500us (0 degrees)
    uint16_t max_pulse; // 2500us (180 or 270 degrees)
    uint16_t max_angle; // 180 or 270
} Servo_t;

extern Servo_t servos[MAX_SERVOS];

void Servo_Init(void);
void Set_Servo_Angle(uint8_t index, uint16_t angle);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
