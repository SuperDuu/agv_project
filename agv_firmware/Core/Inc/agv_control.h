/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    agv_control.h
  * @brief   This file contains all the function prototypes for
  *          the agv_control.c file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AGV_CONTROL_H__
#define __AGV_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motor.h"
#include "sensor.h"
#include <stdbool.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Exported types */
typedef enum {
    MODE_1_LINE_ONLY = 1,
    MODE_2_LINE_INTERSECTION = 2,
    MODE_3_TEST_SENSORS_NO_MOTOR = 3,
    MODE_4_FULL_RUN = 4,
    MODE_5_CALIBRATE_MOTORS = 5,
    MODE_6_TEST_TURN_RIGHT = 6,
    MODE_7_DEBUG_NO_QR = 7,
    MODE_8_TEST_ENCODER = 8
} AGV_RunMode_t;

typedef struct {
    volatile AGV_RunMode_t run_mode;
    volatile uint8_t indicator_state; // 0: Normal, 1: Turning, 2: Error
    volatile bool follow_line_enable;
    volatile bool is_at_intersection;
    volatile uint32_t intersection_time;
    volatile uint32_t last_leave_intersection_time;
    volatile uint16_t current_node;
    volatile uint16_t destination_node;
    volatile bool need_recalculate_path;
    volatile uint32_t last_qr_time;
    volatile uint16_t path_index;
} AGV_State_t;

typedef struct {
    uint32_t time_offset;
    uint32_t time_forward;
    uint32_t time_turn_90;
    uint32_t time_turn_180;
    int16_t turn_speed;
    int16_t base_speed;
} AGV_Config_t;

extern AGV_State_t agv_state;
extern AGV_Config_t agv_config;

typedef struct {
  volatile float current_val;
  volatile float error;
  volatile float prev_error;
  volatile float Kp;
  volatile float Kd;
  volatile float Ki;
  volatile float d;
  volatile float i;
} Pid_data;

typedef struct {
    Motor_HandleTypeDef *motor_left;
    Motor_HandleTypeDef *motor_right;
    LineSensor_HandleTypeDef *line_sensor;
    Pid_data *pid_controller;
    float base_speed;
    float current_speed; // Tốc độ hiện tại (cho Ramping)
    uint8_t direction; // 1: forward, 0: backward
    float current_error;
} AGV_HandleTypeDef;
/* USER CODE END Exported types */

/* USER CODE BEGIN Exported constants */
extern float Delta_t;

#define AGV_LINE_RECOVERY_TIME 1000
#define AGV_TURN_BLIND_TIME 800

/* USER CODE END Exported constants */

/* USER CODE BEGIN Exported macro */

/* USER CODE END Exported macro */

/* Exported functions prototypes ---------------------------------------------*/
void AGV_Init(AGV_HandleTypeDef *hagv, Motor_HandleTypeDef *m_left, Motor_HandleTypeDef *m_right, 
              LineSensor_HandleTypeDef *l_sensor, Pid_data *pid, float base_speed);
float AGV_GetLineError(uint16_t line_value, float current_error);
void AGV_FollowLine(AGV_HandleTypeDef *hagv);
void AGV_Stop(AGV_HandleTypeDef *hagv);
void AGV_TurnLeft(AGV_HandleTypeDef *hagv);
void AGV_TurnRight(AGV_HandleTypeDef *hagv);
void AGV_Turn180(AGV_HandleTypeDef *hagv);

void AGV_TrackLine_Sync(AGV_HandleTypeDef *hagv, uint32_t duration_ms);

void AGV_TurnLeft_IMU(AGV_HandleTypeDef *hagv, uint32_t fwd_delay);
void AGV_TurnRight_IMU(AGV_HandleTypeDef *hagv, uint32_t fwd_delay);
void AGV_Turn180_IMU(AGV_HandleTypeDef *hagv);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __AGV_CONTROL_H__ */
