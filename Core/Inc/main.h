/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "segment_display.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint32_t TimerCounter;
extern uint32_t TimerFlag;
extern uint16_t LEDsFlag;
extern int ADC_BufMean;
extern uint16_t GlobalTempValue;
extern uint16_t CheckLEDsCounter;
extern uint16_t SumValue;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIG4_Pin GPIO_PIN_0
#define DIG4_GPIO_Port GPIOF
#define DIG3_Pin GPIO_PIN_1
#define DIG3_GPIO_Port GPIOF
#define Stop_Pin GPIO_PIN_0
#define Stop_GPIO_Port GPIOA
#define Stop_EXTI_IRQn EXTI0_1_IRQn
#define Start_Pin GPIO_PIN_1
#define Start_GPIO_Port GPIOA
#define Start_EXTI_IRQn EXTI0_1_IRQn
#define CLK_Pin GPIO_PIN_3
#define CLK_GPIO_Port GPIOA
#define DS_Pin GPIO_PIN_4
#define DS_GPIO_Port GPIOA
#define Latch_Pin GPIO_PIN_5
#define Latch_GPIO_Port GPIOA
#define OUTPUT_Pin GPIO_PIN_10
#define OUTPUT_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define TimStart 15
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
