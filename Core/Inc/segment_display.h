/*
 * segment_display.h
 *
 *  Created on: 15 февр. 2023 г.
 *      Author: Irina
 */

#ifndef INC_SEGMENT_DISPLAY_H_
#define INC_SEGMENT_DISPLAY_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Declarations and definitions ----------------------------------------------*/
#define DIGITS_NUM        2
#define REG_NUM      	  3
#define SEGMENTS_NUM      7
#define PIN_ACTIVE        1
#define PIN_DISACTIVE 	  0

typedef struct McuPins
{
  GPIO_TypeDef *port;
  uint16_t pin;
} McuPins;

/* Functions -----------------------------------------------------------------*/
void HC595SendData(uint8_t SendVal);

#endif /* INC_SEGMENT_DISPLAY_H_ */
