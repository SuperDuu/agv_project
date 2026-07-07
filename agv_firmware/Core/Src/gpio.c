/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PH0-OSC_IN(PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT(PH1)   ------> RCC_OSC_OUT
     PA13(JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14(JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, T_ENC_CS3_Pin|T_ENC_CS4_Pin|B_Out0_Pin|B_Out1_Pin
                          |B_Out2_Pin|B_Out15_Pin|T_ENC_CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, B_Out3_Pin|B_Out10_Pin|B_Out11_Pin|B_Out12_Pin
                          |SYS_LAN_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, B_Out4_Pin|B_Out5_Pin|B_Out6_Pin|B_Out7_Pin
                          |SYS_LED2_Pin|SYS_LED3_Pin|B_Out16_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, B_Out13_Pin|T_ADC_CLK_Pin|T_ADC_SDI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, B_Out14_Pin|B_Out17_Pin|B_Out21_Pin|B_Out20_Pin
                          |SYS_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, B_Out23_Pin|B_Out22_Pin|T_DAC_SDI_Pin|T_DAC_CLK_Pin
                          |T_DAC_CS_Pin|T_ENC_CS2_Pin|T_ADC_CS_Pin|T_ADC_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : T_ENC_CS3_Pin T_ENC_CS4_Pin T_ENC_CS1_Pin */
  GPIO_InitStruct.Pin = T_ENC_CS3_Pin|T_ENC_CS4_Pin|T_ENC_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out0_Pin B_Out1_Pin B_Out2_Pin B_Out15_Pin */
  GPIO_InitStruct.Pin = B_Out0_Pin|B_Out1_Pin|B_Out2_Pin|B_Out15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out3_Pin B_Out10_Pin B_Out11_Pin B_Out12_Pin
                           SYS_LAN_CS_Pin */
  GPIO_InitStruct.Pin = B_Out3_Pin|B_Out10_Pin|B_Out11_Pin|B_Out12_Pin
                          |SYS_LAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out4_Pin B_Out5_Pin B_Out6_Pin B_Out7_Pin
                           SYS_LED2_Pin SYS_LED3_Pin B_Out16_Pin */
  GPIO_InitStruct.Pin = B_Out4_Pin|B_Out5_Pin|B_Out6_Pin|B_Out7_Pin
                          |SYS_LED2_Pin|SYS_LED3_Pin|B_Out16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In2_Pin B_In3_Pin B_In4_Pin B_In5_Pin */
  GPIO_InitStruct.Pin = B_In2_Pin|B_In3_Pin|B_In4_Pin|B_In5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : WG1_Pin */
  GPIO_InitStruct.Pin = WG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(WG1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WG0_Pin */
  GPIO_InitStruct.Pin = WG0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(WG0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In10_Pin B_In11_Pin B_In12_Pin B_In13_Pin */
  GPIO_InitStruct.Pin = B_In10_Pin|B_In11_Pin|B_In12_Pin|B_In13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : B_In14_Pin */
  GPIO_InitStruct.Pin = B_In14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B_In14_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out13_Pin T_ADC_CLK_Pin T_ADC_SDI_Pin */
  GPIO_InitStruct.Pin = B_Out13_Pin|T_ADC_CLK_Pin|T_ADC_SDI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : B_In30_Pin */
  GPIO_InitStruct.Pin = B_In30_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B_In30_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In16_Pin B_In26_Pin B_In27_Pin B_In35_Pin
                           B_In34_Pin */
  GPIO_InitStruct.Pin = B_In16_Pin|B_In26_Pin|B_In27_Pin|B_In35_Pin
                          |B_In34_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : B_In15_Pin */
  GPIO_InitStruct.Pin = B_In15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B_In15_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In17_Pin B_In21_Pin B_In20_Pin B_In23_Pin
                           B_In22_Pin B_In25_Pin B_In24_Pin */
  GPIO_InitStruct.Pin = B_In17_Pin|B_In21_Pin|B_In20_Pin|B_In23_Pin
                          |B_In22_Pin|B_In25_Pin|B_In24_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : T_ADC_SDO_Pin */
  GPIO_InitStruct.Pin = T_ADC_SDO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_ADC_SDO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out14_Pin B_Out17_Pin B_Out21_Pin B_Out20_Pin
                           SYS_LED1_Pin */
  GPIO_InitStruct.Pin = B_Out14_Pin|B_Out17_Pin|B_Out21_Pin|B_Out20_Pin
                          |SYS_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Out23_Pin B_Out22_Pin T_DAC_SDI_Pin T_DAC_CLK_Pin
                           T_DAC_CS_Pin T_ADC_CS_Pin T_ADC_RST_Pin */
  GPIO_InitStruct.Pin = B_Out23_Pin|B_Out22_Pin|T_DAC_SDI_Pin|T_DAC_CLK_Pin
                          |T_DAC_CS_Pin|T_ADC_CS_Pin|T_ADC_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : B_In32_Pin B_In31_Pin B_In33_Pin */
  GPIO_InitStruct.Pin = B_In32_Pin|B_In31_Pin|B_In33_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : T_ENC_CS2_Pin */
  GPIO_InitStruct.Pin = T_ENC_CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(T_ENC_CS2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
