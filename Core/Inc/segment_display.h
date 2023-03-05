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

#define ASCII_NUMBER_FIRST_CODE                                             0x30
#define ASCII_NUMBER_LAST_CODE                                              0x39

typedef enum {
  SEG_LCD_OK,
  SEG_LCD_ERROR
} SEG_LCD_Result;

typedef struct McuPins
{
  GPIO_TypeDef *port;
  uint16_t pin;
} McuPin;

/* Functions -----------------------------------------------------------------*/
void HC595SendData(uint8_t SendVal);
/* Functions -----------------------------------------------------------------*/
extern void SEG_LCD_Process();
extern SEG_LCD_Result SEG_LCD_WriteNumber(uint32_t number);
extern SEG_LCD_Result SEG_LCD_WriteString(char* str);


#endif /* INC_SEGMENT_DISPLAY_H_ */
