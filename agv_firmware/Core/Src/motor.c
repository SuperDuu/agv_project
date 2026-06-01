/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.c
  * @brief   This file provides code for the configuration
  *          of the Motor instances.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

void Motor_Init(Motor_HandleTypeDef *hmotor, TIM_HandleTypeDef *htim, uint32_t Channel)
{
    if (hmotor == NULL || htim == NULL) return;
    
    hmotor->htim = htim;
    hmotor->Channel = Channel;
    hmotor->Speed = 0;
    hmotor->Direction = 0;
    
    HAL_TIM_PWM_Start(hmotor->htim, hmotor->Channel);
}

void Motor_SetSpeed(Motor_HandleTypeDef *hmotor, int16_t speed)
{
    if (hmotor == NULL) return;
    
    hmotor->Speed = speed;
    
    uint32_t duty = (speed >= 0) ? speed : -speed;
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->Channel, duty);
}

void Motor_Stop(Motor_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) return;
    
    hmotor->Speed = 0;
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->Channel, 0);
}

/* USER CODE END 1 */
