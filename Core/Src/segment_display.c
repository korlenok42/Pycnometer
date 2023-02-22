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
static McuPins RegPins[REG_NUM] = { {DS_GPIO_Port, DS_Pin},
								   {CLK_GPIO_Port, CLK_Pin},
								   {Latch_GPIO_Port, Latch_Pin} };

//const bool commonCathode = true;


const uint8_t digit_pattern[10] =
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
  0xD6,	  // 0b1011 0110, 5
  0xDE,	  // 0b1011 1110, 6
  0xE0,	  // 0b1110 0000, 7
  0xFE,	  // 0b1111 1110, 8
  0xF2 	  // 0b1111 0110, 9
};

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
    HAL_Delay(100);
    HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
  }

  /* - STEP3, after all the 8-bit data of the shift register is over, the rising edge of the latch clock pin (first pull low level is high) */
  HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_SET);
}
