#include "servo.h"

// Extern declarations of the timer handles defined in main.c
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;

// Global array of servos
Servo_t servos[MAX_SERVOS];

void Servo_Init(void) {
    // Configure the 10 servos mapping to TIM8-TIM12
    // We assume a default of 270 degrees based on the original initialization.
    servos[0] = (Servo_t){&htim8,  TIM_CHANNEL_1, 500, 2500, 270};
    servos[1] = (Servo_t){&htim8,  TIM_CHANNEL_2, 500, 2500, 270};
    servos[2] = (Servo_t){&htim8,  TIM_CHANNEL_3, 500, 2500, 270};
    servos[3] = (Servo_t){&htim8,  TIM_CHANNEL_4, 500, 2500, 270};
    servos[4] = (Servo_t){&htim9,  TIM_CHANNEL_1, 500, 2500, 270};
    servos[5] = (Servo_t){&htim9,  TIM_CHANNEL_2, 500, 2500, 270};
    servos[6] = (Servo_t){&htim10, TIM_CHANNEL_1, 500, 2500, 270};
    servos[7] = (Servo_t){&htim11, TIM_CHANNEL_1, 500, 2500, 270};
    servos[8] = (Servo_t){&htim12, TIM_CHANNEL_1, 500, 2500, 270};
    servos[9] = (Servo_t){&htim12, TIM_CHANNEL_2, 500, 2500, 270};

    // Start PWM for all 10 servo channels and set initial neutral position (1500us / 135 degrees)
    for (int i = 0; i < MAX_SERVOS; i++) {
        HAL_TIM_PWM_Start(servos[i].htim, servos[i].channel);
        Set_Servo_Angle(i, servos[i].max_angle / 2); // Set to neutral position
    }
}

void Set_Servo_Angle(uint8_t index, uint16_t angle) {
    if (index >= MAX_SERVOS) return;
    if (angle > servos[index].max_angle) angle = servos[index].max_angle;
    
    // Linearly map the angle to the pulse width (500us - 2500us)
    uint32_t pulse = servos[index].min_pulse + 
                     ((uint32_t)angle * (servos[index].max_pulse - servos[index].min_pulse) / servos[index].max_angle);
                     
    __HAL_TIM_SET_COMPARE(servos[index].htim, servos[index].channel, pulse);
}
