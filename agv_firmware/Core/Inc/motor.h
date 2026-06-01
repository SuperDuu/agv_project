/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.h
  * @brief   This file contains all the function prototypes for
  *          the motor.c file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Exported types */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t Channel;
    int16_t Speed;
    uint8_t Direction;
} Motor_HandleTypeDef;
/* USER CODE END Exported types */

/* USER CODE BEGIN Exported constants */

/* USER CODE END Exported constants */

/* USER CODE BEGIN Exported macro */

/* USER CODE END Exported macro */

/* Exported functions prototypes ---------------------------------------------*/
void Motor_Init(Motor_HandleTypeDef *hmotor, TIM_HandleTypeDef *htim, uint32_t Channel);
void Motor_SetSpeed(Motor_HandleTypeDef *hmotor, int16_t speed);
void Motor_Stop(Motor_HandleTypeDef *hmotor);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
