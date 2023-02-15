/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

uint32_t TimerFlag = 0;
uint16_t GlobalTempValue = 0;
uint16_t LEDsFlag = 0;
uint16_t SumValue = 0;
uint16_t CheckLEDsCounter = 0;
int ADC_BufMean = 0;  					// get average ADC val
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t ReadADCvalue;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == Start_Pin) TimerFlag = 1;

  else if(GPIO_Pin == Stop_Pin) {
	  TimerFlag = 0;
	  TimerCounter = TimStart;
	  HAL_GPIO_WritePin(OUTPUT_GPIO_Port, OUTPUT_Pin, RESET);
  }

//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);}
    else{

    __NOP();

  }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  uint16_t StartValue = 5;
  uint16_t pos, neg;

  // Calibrate The ADC On Power-Up For Better Accuracy
  HAL_ADCEx_Calibration_Start(&hadc);
  // Start ADC Conversion
  HAL_ADC_Start(&hadc);

  HAL_TIM_Base_Start_IT(&htim3);

  HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, SET);
  HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	SEG_LCD_Process();
	HAL_Delay(1);

	if(!TimerFlag) {

		// Poll ADC1 Perihperal & TimeOut = 10mSec
		HAL_ADC_PollForConversion(ADC, 1000);

		// Read The ADC Conversion Result
		for(int i = 0; i <= ADC_Operations; i++)
		{
			ADC_Buf[i] = HAL_ADC_GetValue(ADC);
		}

		ADC_AverageMean();

		// start = 10 ; stop = 20
		//StartValue = (uint32_t) 10 + ( ADC_BufMean / 400);

		// start = 5 ; stop = 20
		GlobalTempValue = (uint32_t) 5 + ( ADC_BufMean / 270);

		if (LEDsFlag)
		{
			StartValue = SumValue/16;
			TimerCounter = StartValue;
			SEG_LCD_WriteNumber(TimerCounter);
			LEDsFlag = 0;
		}

/*		CheckNumber(ADC_BufMean);

		if(SetLedFlag)
		{
			StartValue = 5 + GlobalTempValue;
			SetLedFlag = 0;
		}
		//StartValue += TempValue;


		//StartValue = StartValue / 4;

		TimerCounter = StartValue;*/

	}

	else
	{
		if (TimerCounter == StartValue) HAL_GPIO_WritePin(OUTPUT_GPIO_Port, OUTPUT_Pin, SET);

		if (TimerCounter == 0) {

	  			TimerCounter = TimStart;
	  			TimerFlag = 0;
	  			HAL_GPIO_WritePin(OUTPUT_GPIO_Port, OUTPUT_Pin, RESET);
	  		}

	  	SEG_LCD_WriteNumber(TimerCounter);
	  }

	//SEG_LCD_WriteNumber(TimerCounter);

/*		currentTime = HAL_GetTick();

		if ((currentTime - previousTime) >= outputPeriod)
		{
		    if (outputValue == outputLimit)
		  	{
		    	outputValue = 15;
			}
	        SEG_LCD_WriteNumber(outputValue);

	        outputValue -= outputStep;

		  	previousTime = currentTime;
		}
*/
/*	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, 1);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, 1);
	HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, 1);
	HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, 1);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, 1);
	HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, 1);
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, 1);

	HAL_GPIO_WritePin(DIG4_GPIO_Port, DIG4_Pin, 0);
	HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, 1);
*/
// 	TESTS
//  HAL_GPIO_TogglePin(C_GPIO_Port, C_Pin);
//  HAL_Delay(1000);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, DIG4_Pin|DIG3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, A_Pin|B_Pin|C_Pin|E_Pin
                          |D_Pin|F_Pin|G_Pin|OUTPUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIG4_Pin DIG3_Pin */
  GPIO_InitStruct.Pin = DIG4_Pin|DIG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : Stop_Pin */
  GPIO_InitStruct.Pin = Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Stop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Start_Pin */
  GPIO_InitStruct.Pin = Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Start_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A_Pin B_Pin C_Pin E_Pin
                           D_Pin F_Pin G_Pin OUTPUT_Pin */
  GPIO_InitStruct.Pin = A_Pin|B_Pin|C_Pin|E_Pin
                          |D_Pin|F_Pin|G_Pin|OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
