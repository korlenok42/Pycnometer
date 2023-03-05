/*
 * segment_lcd.c
 *
 *  Created on: Nov 11, 2022
 *      Author: Irina
 */

/* Includes ------------------------------------------------------------------*/
#include "segment_lcd.h"
#include <stdio.h>
/* Declarations and definitions ----------------------------------------------*/
static McuPin digitPins[DIGITS_NUM] = { {DIG3_GPIO_Port, DIG3_Pin}, {DIG4_GPIO_Port, DIG4_Pin}};

static McuPin segmentPins[SEGMENTS_NUM] = { {GPIOA, GPIO_PIN_2}, {GPIOA, GPIO_PIN_3},
                                            {GPIOA, GPIO_PIN_4}, {GPIOA, GPIO_PIN_6},
                                            {GPIOA, GPIO_PIN_5}, {GPIOA, GPIO_PIN_7},
                                            {GPIOA, GPIO_PIN_9} };

//static McuPin dotPin = {GPIOB, GPIO_PIN_12};

static uint8_t charactersTable[DIGIT_CHARACTERS_NUM + EXTRA_CHARACTERS_NUM] =
               {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x40, 0x00};

static SEG_LCD_ExtraCharacter extraCharacters[EXTRA_CHARACTERS_NUM] = { {ASCII_MINUS_CODE, DIGIT_CHARACTERS_NUM},
                                                                        {ASCII_SPACE_CODE, DIGIT_CHARACTERS_NUM + 1} };

// выбор цифры
static uint8_t currentCharacters[DIGITS_NUM] = {0x00, 0x00};
static uint8_t currentDots[DIGITS_NUM] = {0, 0};
static uint8_t currentDigitIndex = 0;
/* Functions -----------------------------------------------------------------*/
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
    if (*str == ASCII_DOT_CODE)
    {
      if (currentDigitIndex > 0)
      {
        currentDots[currentDigitIndex - 1] = 1;
      }
    }
    else
    {
      if ((*str >= ASCII_NUMBER_FIRST_CODE) && (*str <= ASCII_NUMBER_LAST_CODE))
      {
        uint8_t currentCharacterIndex = (*str - ASCII_NUMBER_FIRST_CODE);
        currentCharacters[currentDigitIndex] = charactersTable[currentCharacterIndex];
        currentDigitIndex++;
      }
      else
      {
        uint8_t found = 0;
        for (uint8_t i = 0; i < EXTRA_CHARACTERS_NUM; i++)
        {
          if (*str == extraCharacters[i].asciiCode)
          {
            uint8_t currentCharacterIndex = extraCharacters[i].symbolsTableOffset;
            currentCharacters[currentDigitIndex] = charactersTable[currentCharacterIndex];
            found = 1;
            currentDigitIndex++;
            break;
          }
        }
        if (found == 0)
        {
          return SEG_LCD_ERROR;
        }
      }
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
      currentDots[i + (DIGITS_NUM - currentDigitIndex)] = currentDots[i];
    }
    for (uint8_t i = 0; i < (DIGITS_NUM - currentDigitIndex); i++)
    {
      currentCharacters[i] = 0x00;
      currentDots[i] = 0;
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
static void SetOutput(McuPin output, uint8_t state)
{
  HAL_GPIO_WritePin(output.port, output.pin, (GPIO_PinState)state);
}
/*----------------------------------------------------------------------------*/
static void SetSegmentPins(uint8_t characterCode)
{
  for (uint8_t i = 0; i < SEGMENTS_NUM; i++)
  {
    uint8_t bit = (characterCode >> i) & 0x01;
    if (bit == 1)
    {
      SetOutput(segmentPins[i], SEGMENT_PIN_ACTIVE);
    }
    else
    {
      SetOutput(segmentPins[i], !SEGMENT_PIN_ACTIVE);
    }
  }
}
/*----------------------------------------------------------------------------*/
void SEG_LCD_Process()
{
  // выбор земленного пина (на землю)
  for (uint8_t i = 0; i < DIGITS_NUM; i++)
  {
    SetOutput(digitPins[i], !DIGIT_PIN_ACTIVE);
  }

  // выбор значения на индикаторе
  SetSegmentPins(currentCharacters[currentDigitIndex]);
/*  if (currentDots[currentDigitIndex] == 1)
  {
    SetOutput(dotPin, SEGMENT_PIN_ACTIVE);
  }
  else
  {
    SetOutput(dotPin, !SEGMENT_PIN_ACTIVE);
  }*/

  SetOutput(digitPins[currentDigitIndex], DIGIT_PIN_ACTIVE);
  currentDigitIndex++;
  if (currentDigitIndex == DIGITS_NUM)
  {
    currentDigitIndex = 0;
  }
}
/*----------------------------------------------------------------------------*/

