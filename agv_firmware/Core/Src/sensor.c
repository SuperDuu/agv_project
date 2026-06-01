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

void LineSensor_Init(LineSensor_HandleTypeDef *hline, GPIO_TypeDef **ports, uint16_t *pins)
{
    if (hline == NULL || ports == NULL || pins == NULL) return;
    
    for (int i = 0; i < 16; i++) {
        Sensor_Init(&hline->sensors[i], ports[i], pins[i]);
    }
    hline->line_value = 0;
}

uint16_t LineSensor_Read(LineSensor_HandleTypeDef *hline)
{
    if (hline == NULL) return 0;
    
    hline->line_value = 0;
    for (int i = 0; i < 16; i++) {
        if (Sensor_Read(&hline->sensors[i])) {
            // Map sensor[0] to MSB (bit 15) and sensor[15] to LSB (bit 0) 
            // to match the string representation "1111110000111111"
            hline->line_value |= (1 << (15 - i));
        }
    }
    return hline->line_value;
}

/* USER CODE END 1 */
