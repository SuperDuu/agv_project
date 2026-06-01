/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "sensor.h"
#include "agv_control.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define T_ENC_CS3_Pin GPIO_PIN_2
#define T_ENC_CS3_GPIO_Port GPIOE
#define T_ENC_CS4_Pin GPIO_PIN_3
#define T_ENC_CS4_GPIO_Port GPIOE
#define B_Out0_Pin GPIO_PIN_4
#define B_Out0_GPIO_Port GPIOE
#define B_Out1_Pin GPIO_PIN_5
#define B_Out1_GPIO_Port GPIOE
#define B_Out3_Pin GPIO_PIN_13
#define B_Out3_GPIO_Port GPIOC
#define B_In0_Pin GPIO_PIN_1
#define B_In0_GPIO_Port GPIOF
#define B_Out4_Pin GPIO_PIN_2
#define B_Out4_GPIO_Port GPIOF
#define B_Out5_Pin GPIO_PIN_3
#define B_Out5_GPIO_Port GPIOF
#define B_In1_Pin GPIO_PIN_4
#define B_In1_GPIO_Port GPIOF
#define B_In2_Pin GPIO_PIN_5
#define B_In2_GPIO_Port GPIOF
#define B_In3_Pin GPIO_PIN_6
#define B_In3_GPIO_Port GPIOF
#define B_In4_Pin GPIO_PIN_7
#define B_In4_GPIO_Port GPIOF
#define B_In5_Pin GPIO_PIN_8
#define B_In5_GPIO_Port GPIOF
#define B_Out6_Pin GPIO_PIN_9
#define B_Out6_GPIO_Port GPIOF
#define B_Out7_Pin GPIO_PIN_10
#define B_Out7_GPIO_Port GPIOF
#define B_Out10_Pin GPIO_PIN_0
#define B_Out10_GPIO_Port GPIOC
#define B_Out11_Pin GPIO_PIN_1
#define B_Out11_GPIO_Port GPIOC
#define B_Out12_Pin GPIO_PIN_2
#define B_Out12_GPIO_Port GPIOC
#define B_In6_Pin GPIO_PIN_3
#define B_In6_GPIO_Port GPIOC
#define B_In7_Pin GPIO_PIN_0
#define B_In7_GPIO_Port GPIOA
#define B_In10_Pin GPIO_PIN_1
#define B_In10_GPIO_Port GPIOA
#define B_In11_Pin GPIO_PIN_2
#define B_In11_GPIO_Port GPIOA
#define B_In12_Pin GPIO_PIN_3
#define B_In12_GPIO_Port GPIOA
#define B_In13_Pin GPIO_PIN_4
#define B_In13_GPIO_Port GPIOA
#define SYS_LAN_CLK_Pin GPIO_PIN_5
#define SYS_LAN_CLK_GPIO_Port GPIOA
#define SYS_LAN_SDO_Pin GPIO_PIN_6
#define SYS_LAN_SDO_GPIO_Port GPIOA
#define SYS_LAN_SDI_Pin GPIO_PIN_7
#define SYS_LAN_SDI_GPIO_Port GPIOA
#define SYS_LAN_CS_Pin GPIO_PIN_4
#define SYS_LAN_CS_GPIO_Port GPIOC
#define B_In14_Pin GPIO_PIN_5
#define B_In14_GPIO_Port GPIOC
#define B_Out13_Pin GPIO_PIN_0
#define B_Out13_GPIO_Port GPIOB
#define SYS_LED2_Pin GPIO_PIN_11
#define SYS_LED2_GPIO_Port GPIOF
#define SYS_LED3_Pin GPIO_PIN_12
#define SYS_LED3_GPIO_Port GPIOF
#define B_In30_Pin GPIO_PIN_14
#define B_In30_GPIO_Port GPIOF
#define B_Out16_Pin GPIO_PIN_15
#define B_Out16_GPIO_Port GPIOF
#define B_In16_Pin GPIO_PIN_0
#define B_In16_GPIO_Port GPIOG
#define B_In15_Pin GPIO_PIN_1
#define B_In15_GPIO_Port GPIOG
#define B_Out2_Pin GPIO_PIN_7
#define B_Out2_GPIO_Port GPIOE
#define B_In17_Pin GPIO_PIN_8
#define B_In17_GPIO_Port GPIOE
#define B_In21_Pin GPIO_PIN_9
#define B_In21_GPIO_Port GPIOE
#define B_In20_Pin GPIO_PIN_10
#define B_In20_GPIO_Port GPIOE
#define B_In23_Pin GPIO_PIN_11
#define B_In23_GPIO_Port GPIOE
#define B_In22_Pin GPIO_PIN_12
#define B_In22_GPIO_Port GPIOE
#define B_In25_Pin GPIO_PIN_13
#define B_In25_GPIO_Port GPIOE
#define B_In24_Pin GPIO_PIN_14
#define B_In24_GPIO_Port GPIOE
#define B_Out15_Pin GPIO_PIN_15
#define B_Out15_GPIO_Port GPIOE
#define T_ADC_CLK_Pin GPIO_PIN_13
#define T_ADC_CLK_GPIO_Port GPIOB
#define T_ADC_SDO_Pin GPIO_PIN_14
#define T_ADC_SDO_GPIO_Port GPIOB
#define T_ADC_SDI_Pin GPIO_PIN_15
#define T_ADC_SDI_GPIO_Port GPIOB
#define B_Out14_Pin GPIO_PIN_8
#define B_Out14_GPIO_Port GPIOD
#define B_Out17_Pin GPIO_PIN_9
#define B_Out17_GPIO_Port GPIOD
#define B_Out21_Pin GPIO_PIN_10
#define B_Out21_GPIO_Port GPIOD
#define B_Out20_Pin GPIO_PIN_11
#define B_Out20_GPIO_Port GPIOD
#define T_PWM4_Pin GPIO_PIN_12
#define T_PWM4_GPIO_Port GPIOD
#define T_PWM3_Pin GPIO_PIN_13
#define T_PWM3_GPIO_Port GPIOD
#define T_PWM2_Pin GPIO_PIN_14
#define T_PWM2_GPIO_Port GPIOD
#define T_PWM1_Pin GPIO_PIN_15
#define T_PWM1_GPIO_Port GPIOD
#define B_Out23_Pin GPIO_PIN_2
#define B_Out23_GPIO_Port GPIOG
#define B_Out22_Pin GPIO_PIN_3
#define B_Out22_GPIO_Port GPIOG
#define B_In26_Pin GPIO_PIN_4
#define B_In26_GPIO_Port GPIOG
#define B_In27_Pin GPIO_PIN_5
#define B_In27_GPIO_Port GPIOG
#define T_DAC_SDI_Pin GPIO_PIN_6
#define T_DAC_SDI_GPIO_Port GPIOG
#define T_DAC_CLK_Pin GPIO_PIN_7
#define T_DAC_CLK_GPIO_Port GPIOG
#define T_DAC_CS_Pin GPIO_PIN_8
#define T_DAC_CS_GPIO_Port GPIOG
#define T_PWM8_Pin GPIO_PIN_6
#define T_PWM8_GPIO_Port GPIOC
#define T_PWM7_Pin GPIO_PIN_7
#define T_PWM7_GPIO_Port GPIOC
#define T_PWM6_Pin GPIO_PIN_8
#define T_PWM6_GPIO_Port GPIOC
#define T_PWM5_Pin GPIO_PIN_9
#define T_PWM5_GPIO_Port GPIOC
#define COM_RS232_TX_Pin GPIO_PIN_9
#define COM_RS232_TX_GPIO_Port GPIOA
#define COM_RS232_RX_Pin GPIO_PIN_10
#define COM_RS232_RX_GPIO_Port GPIOA
#define COM_RS485_TX0_Pin GPIO_PIN_10
#define COM_RS485_TX0_GPIO_Port GPIOC
#define COM_RS485_RX0_Pin GPIO_PIN_11
#define COM_RS485_RX0_GPIO_Port GPIOC
#define COM_RS485_TX2_Pin GPIO_PIN_12
#define COM_RS485_TX2_GPIO_Port GPIOC
#define SYS_LED1_Pin GPIO_PIN_0
#define SYS_LED1_GPIO_Port GPIOD
#define COM_RS485_RX2_Pin GPIO_PIN_2
#define COM_RS485_RX2_GPIO_Port GPIOD
#define B_In32_Pin GPIO_PIN_3
#define B_In32_GPIO_Port GPIOD
#define B_In31_Pin GPIO_PIN_4
#define B_In31_GPIO_Port GPIOD
#define COM_RS485_TX1_Pin GPIO_PIN_5
#define COM_RS485_TX1_GPIO_Port GPIOD
#define COM_RS485_RX1_Pin GPIO_PIN_6
#define COM_RS485_RX1_GPIO_Port GPIOD
#define B_In33_Pin GPIO_PIN_7
#define B_In33_GPIO_Port GPIOD
#define T_ENC_CS2_Pin GPIO_PIN_10
#define T_ENC_CS2_GPIO_Port GPIOG
#define T_ADC_CS_Pin GPIO_PIN_12
#define T_ADC_CS_GPIO_Port GPIOG
#define T_ADC_RST_Pin GPIO_PIN_13
#define T_ADC_RST_GPIO_Port GPIOG
#define B_In35_Pin GPIO_PIN_14
#define B_In35_GPIO_Port GPIOG
#define B_In34_Pin GPIO_PIN_15
#define B_In34_GPIO_Port GPIOG
#define T_ENC_CLK_Pin GPIO_PIN_3
#define T_ENC_CLK_GPIO_Port GPIOB
#define T_ENC_SDO_Pin GPIO_PIN_4
#define T_ENC_SDO_GPIO_Port GPIOB
#define T_ENC_SDI_Pin GPIO_PIN_5
#define T_ENC_SDI_GPIO_Port GPIOB
#define T_ENC_CS1_Pin GPIO_PIN_0
#define T_ENC_CS1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
