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

void Motor_Init(Motor_HandleTypeDef *hmotor, TIM_HandleTypeDef *htim, uint32_t Channel,
                GPIO_TypeDef *DIR_Port, uint16_t DIR_Pin,
                GPIO_TypeDef *EN_Port, uint16_t EN_Pin)
{
    if (hmotor == NULL || htim == NULL) return;
    
    hmotor->htim = htim;
    hmotor->Channel = Channel;
    hmotor->DIR_Port = DIR_Port;
    hmotor->DIR_Pin = DIR_Pin;
    hmotor->EN_Port = EN_Port;
    hmotor->EN_Pin = EN_Pin;
    
    hmotor->Speed = 0;
    hmotor->Direction = 0;
    hmotor->InvertDirection = 0;
    
    if (hmotor->EN_Port != NULL) {
        HAL_GPIO_WritePin(hmotor->EN_Port, hmotor->EN_Pin, GPIO_PIN_SET); 
    }
    
    HAL_TIM_PWM_Start(hmotor->htim, hmotor->Channel);
}

void Motor_SetSpeed(Motor_HandleTypeDef *hmotor, int16_t speed)
{
    if (hmotor == NULL) return;
    
    int16_t physical_speed = hmotor->InvertDirection ? -speed : speed;
    hmotor->Speed = speed;
    
    if (physical_speed >= 0) {
        hmotor->Direction = 1;
        if (hmotor->DIR_Port != NULL) {
            HAL_GPIO_WritePin(hmotor->DIR_Port, hmotor->DIR_Pin, GPIO_PIN_SET);
        }
    } else {
        hmotor->Direction = 0;
        if (hmotor->DIR_Port != NULL) {
            HAL_GPIO_WritePin(hmotor->DIR_Port, hmotor->DIR_Pin, GPIO_PIN_RESET);
        }
    }
    
    uint32_t duty = (physical_speed >= 0) ? physical_speed : -physical_speed;
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->Channel, duty);
}

void Motor_Stop(Motor_HandleTypeDef *hmotor)
{
    if (hmotor == NULL) return;
    hmotor->Speed = 0;
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->Channel, 0);
}

/* USER CODE END 1 */
