/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sensor.c
  * @brief   This file provides code for the configuration
  *          of the Sensor instances.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sensor.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

void Sensor_Init(Sensor_HandleTypeDef *hsensor, GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin)
{
    if (hsensor == NULL) return;
    
    hsensor->GPIO_Port = GPIO_Port;
    hsensor->GPIO_Pin = GPIO_Pin;
    hsensor->State = 0;
}

uint8_t Sensor_Read(Sensor_HandleTypeDef *hsensor)
{
    if (hsensor == NULL) return 0;
    
    hsensor->State = HAL_GPIO_ReadPin(hsensor->GPIO_Port, hsensor->GPIO_Pin);
    return hsensor->State;
}

/* USER CODE END 1 */
