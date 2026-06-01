/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sensor.h
  * @brief   This file contains all the function prototypes for
  *          the sensor.c file
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSOR_H__
#define __SENSOR_H__

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
    GPIO_TypeDef *GPIO_Port;
    uint16_t GPIO_Pin;
    uint8_t State;
} Sensor_HandleTypeDef;
/* USER CODE END Exported types */

/* USER CODE BEGIN Exported constants */

/* USER CODE END Exported constants */

/* USER CODE BEGIN Exported macro */

/* USER CODE END Exported macro */

/* Exported functions prototypes ---------------------------------------------*/
void Sensor_Init(Sensor_HandleTypeDef *hsensor, GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin);
uint8_t Sensor_Read(Sensor_HandleTypeDef *hsensor);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_H__ */
