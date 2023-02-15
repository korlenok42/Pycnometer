/*
 * segment_lcd.h
 *
 *  Created on: Nov 11, 2022
 *      Author: Irina
 */

#ifndef INC_SEGMENT_LCD_H_
#define INC_SEGMENT_LCD_H_

/* Includes ------------------------------------------------------------------*/
//#include "stm32f1xx_hal.h"
#include "main.h"

/* Declarations and definitions ----------------------------------------------*/
#define DIGITS_NUM                                                          2
#define SEGMENTS_NUM                                                        7
#define SEGMENT_PIN_ACTIVE                                                  1
#define DIGIT_PIN_ACTIVE                                                    !SEGMENT_PIN_ACTIVE
#define DIGIT_CHARACTERS_NUM                                                10
#define EXTRA_CHARACTERS_NUM                                                2
#define ASCII_NUMBER_FIRST_CODE                                             0x30
#define ASCII_NUMBER_LAST_CODE                                              0x39
#define ASCII_MINUS_CODE                                                    0x2D
#define ASCII_SPACE_CODE                                                    0x20
#define ASCII_DOT_CODE                                                      0x2E
typedef enum {
  SEG_LCD_OK,
  SEG_LCD_ERROR
} SEG_LCD_Result;
typedef struct SEG_LCD_ExtraCharacter
{
  uint8_t asciiCode;
  uint8_t symbolsTableOffset;
} SEG_LCD_ExtraCharacter;
typedef struct McuPin
{
  GPIO_TypeDef *port;
  uint16_t pin;
} McuPin;
/* Functions -----------------------------------------------------------------*/
extern void SEG_LCD_Process();
extern SEG_LCD_Result SEG_LCD_WriteNumber(uint32_t number);
extern SEG_LCD_Result SEG_LCD_WriteString(char* str);

#endif /* INC_SEGMENT_LCD_H_ */
