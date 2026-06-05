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
    MODE_7_DEBUG_NO_QR = 7
} AGV_RunMode_t;

extern volatile AGV_RunMode_t agv_run_mode;
typedef struct {
  volatile float gtht;
  volatile float er;
  volatile float pre_er;
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
    uint8_t direction; // 1: forward, 0: backward
    float current_error;
} AGV_HandleTypeDef;
/* USER CODE END Exported types */

/* USER CODE BEGIN Exported constants */
extern float Delta_t;
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

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __AGV_CONTROL_H__ */
