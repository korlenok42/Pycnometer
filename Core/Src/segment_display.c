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
#include "segment_lcd.h"
/* Declarations and definitions ----------------------------------------------*/

//const bool commonCathode = true;

/*
const byte digit_pattern[10] =
{
  // 74HC595 Outpin Connection with 7segment display.
  // Q0 Q1 Q2 Q3 Q4 Q5 Q6
  // a  b  c  d  e  f  g
  0b11111100,  // 0
  0b01100000,  // 1
  0b11011010,  // 2
  0b11110010,  // 3
  0b01100110,  // 4
  0b10110110,  // 5
  0b10111110,  // 6
  0b11100000,  // 7
  0b11111110,  // 8
  0b11110110,  // 9
};
*/
/* Functions -----------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

