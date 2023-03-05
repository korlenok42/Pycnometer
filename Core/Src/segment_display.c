/*
 * segment_display.c
 *
 *  Created on: 15 февр. 2023 г.
 *      Author: Irina
 *
 *  Comment:
 *  Hardware Connections:
 *  Vin  - 5V (3.3V is allowed)
 * 	GND - GNDs
 * 	74HC595 ST_CP - 11 PA5 (STM32F030) Latch
 * 	74HC595 SH_CP - 9  PA3 (STM32F030) CLK
 * 	74HC595 DS 	  - 10 PA4 (STM32F030)
 */



/* Includes ------------------------------------------------------------------*/
#include "segment_display.h"
/* Declarations and definitions ----------------------------------------------*/
static McuPin RegPins[REG_NUM] = { {DS_GPIO_Port, DS_Pin},
								   {CLK_GPIO_Port, CLK_Pin},
								   {Latch_GPIO_Port, Latch_Pin} };

static McuPin digitPins[DIGITS_NUM] = { {DIG3_GPIO_Port, DIG3_Pin}, {DIG4_GPIO_Port, DIG4_Pin}};

//const bool commonCathode = true;


const uint8_t charactersTable[10] =
{
  // 0000 0010 - g
  // 0000 0100 - f
  // 0000 1000 - e
  // 0001 0000 - d
  // 0010 0000 - c
  // 0100 0000 - b
  // 1000 0000 - a

  // 74HC595 output pin Connection with 7segment display.
  // Q0 Q1 Q2 Q3 Q4 Q5 Q6
  // a  b  c  d  e  f  g
  0xFC,   // 0b1111 1100, 0
  0x60,	  // 0b0110 0000, 1
  0xDA,	  // 0b1101 1010, 2
  0xF2,	  // 0b1111 0010, 3
  0x66,	  // 0b0110 0110, 4
  0xB6,	  // 0b1011 0110, 5
  0xBE,	  // 0b1011 1110, 6
  0xE0,	  // 0b1110 0000, 7
  0xFE,	  // 0b1111 1110, 8
  0xF6 	  // 0b1111 0110, 9
};

// выбор цифры
static uint8_t currentCharacters[DIGITS_NUM] = {0x00, 0x00};
static uint8_t currentDots[DIGITS_NUM] = {0, 0};
static uint8_t currentDigitIndex = 0;

/* Functions -----------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
void HC595SendData(uint8_t SendVal)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    /* - STEP1, serial input pin*/
    if ((SendVal & (1 << i)) != 0 )
      HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);

    /* - STEP2, SHCP occurs once, 74HC595 will get current data from the DS pin */
    HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
  }

  /* - STEP3, after all the 8-bit data of the shift register is over, the rising edge of the latch clock pin (first pull low level is high) */
  HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_SET);
}

/*----------------------------------------------------------------------------*/
static void SetOutput(McuPin output, uint8_t state)
{
  HAL_GPIO_WritePin(output.port, output.pin, (GPIO_PinState)state);
}
/*----------------------------------------------------------------------------*/
SEG_LCD_Result SEG_LCD_WriteString(char* str)
{
  uint8_t currentDigitIndex = 0;
  for (uint8_t i = 0; i < DIGITS_NUM; i++)
  {
    currentCharacters[i] = 0x00;
    currentDots[i] = 0;
  }
  while(*str != '\0')
  {
	  if ((*str >= ASCII_NUMBER_FIRST_CODE) && (*str <= ASCII_NUMBER_LAST_CODE))
	  {
		  uint8_t currentCharacterIndex = (*str - ASCII_NUMBER_FIRST_CODE);
		  currentCharacters[currentDigitIndex] = charactersTable[currentCharacterIndex];
		  currentDigitIndex++;
	  }

	  if (currentDigitIndex == DIGITS_NUM)
	  {
		 break;
	  }
	  str++;
  }

  if (currentDigitIndex < DIGITS_NUM)
  {
    for (int8_t i = currentDigitIndex - 1; i >= 0; i--)
    {
      currentCharacters[i + (DIGITS_NUM - currentDigitIndex)] = currentCharacters[i];
    }
    for (uint8_t i = 0; i < (DIGITS_NUM - currentDigitIndex); i++)
    {
      currentCharacters[i] = 0x00;
    }
  }
  return SEG_LCD_OK;
}
/*----------------------------------------------------------------------------*/
SEG_LCD_Result SEG_LCD_WriteNumber(uint32_t number)
{
  char temp[DIGITS_NUM+2];
  snprintf(temp, DIGITS_NUM+2, "%d", number);



  SEG_LCD_WriteString(temp);

  return SEG_LCD_OK;
}

/*----------------------------------------------------------------------------*/
void SEG_LCD_Process()
{
  // выбор земленного пина (на землю)
/*  for (uint8_t i = 0; i < DIGITS_NUM; i++)
  {
    SetOutput(digitPins[i], PIN_ACTIVE);
	//HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, SET);
  }*/

  // выбор значения на индикаторе
  HC595SendData(currentCharacters[currentDigitIndex]);


  //HAL_GPIO_WritePin(DIG3_GPIO_Port, DIG3_Pin, RESET);
  //SetOutput(digitPins[currentDigitIndex], !PIN_ACTIVE);

  HAL_GPIO_TogglePin(DIG3_GPIO_Port, DIG3_Pin);
  HAL_GPIO_TogglePin(DIG4_GPIO_Port, DIG4_Pin);
  currentDigitIndex++;
  if (currentDigitIndex == DIGITS_NUM)
  {
    currentDigitIndex = 0;
  }
}
/*----------------------------------------------------------------------------*/
