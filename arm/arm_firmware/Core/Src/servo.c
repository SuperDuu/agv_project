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

void Servo_Test_Patterns(void) {
    // -------------------------------------------------------------------------
    // Pattern 1: Dynamic Sweep (All channels sweep together)
    // -------------------------------------------------------------------------
    // Sweeps all servos from 0 to 270 degrees, then back.
    for (uint16_t angle = 0; angle <= 270; angle += 2) {
        for (uint8_t i = 0; i < MAX_SERVOS; i++) {
            Set_Servo_Angle(i, angle);
        }
        HAL_Delay(15);
    }
    HAL_Delay(500);

    for (int16_t angle = 270; angle >= 0; angle -= 2) {
        for (uint8_t i = 0; i < MAX_SERVOS; i++) {
            Set_Servo_Angle(i, (uint16_t)angle);
        }
        HAL_Delay(15);
    }

    HAL_Delay(1000);

    // -------------------------------------------------------------------------
    // Pattern 2: Channel Identification (Unique static duty cycle per channel)
    // -------------------------------------------------------------------------
    // Set each channel to a distinct angle so you can measure/identify each:
    // Ch 0: 0 deg (500us)    | Ch 5: 150 deg (1611us)
    // Ch 1: 30 deg (722us)   | Ch 6: 180 deg (1833us)
    // Ch 2: 60 deg (944us)   | Ch 7: 210 deg (2055us)
    // Ch 3: 90 deg (1166us)  | Ch 8: 240 deg (2277us)
    // Ch 4: 120 deg (1388us) | Ch 9: 270 deg (2500us)
    // -------------------------------------------------------------------------
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        uint16_t test_angle = i * 30; // 0 to 270 degrees
        Set_Servo_Angle(i, test_angle);
    }
    HAL_Delay(5000); // Hold for 5 seconds

    // -------------------------------------------------------------------------
    // Pattern 3: Discrete Limit Test (All MIN, all MID, all MAX)
    // -------------------------------------------------------------------------
    // All MIN (0 deg / 500us)
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        Set_Servo_Angle(i, 0);
    }
    HAL_Delay(2000);

    // All MID (135 deg / 1500us)
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        Set_Servo_Angle(i, 135);
    }
    HAL_Delay(2000);

    // All MAX (270 deg / 2500us)
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        Set_Servo_Angle(i, 270);
    }
    HAL_Delay(2000);
}

