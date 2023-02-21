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


const byte digit_pattern[10] =
{
  // 74HC595 Outpin Connection with 7segment display.
  // Q0 Q1 Q2 Q3 Q4 Q5 Q6
  // a  b  c  d  e  f  g
  0xFC,   // 0b1111 1100, 0
  0x60,	  // 0b0110 0000, 1
  oxDA,	  // 0b1101 1010, 2
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

