/*
 * ADC.h
 *
 *  Created on: 13 нояб. 2022 г.
 *      Author: Irina
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"

/* ---------------------------------------------------------------------------*/
ADC_HandleTypeDef hadc;

/* Private defines -----------------------------------------------------------*/
#define ADC &hadc

#define ADC_ChannelAmount 	1
#define ADC_Operations 		15


/* ---------------------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
void ADC_AverageMean (void);
//uint16_t FinalNumber (uint16_t counts);
void CheckNumber (uint16_t counts);
/* ---------------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

extern int ADC_Buf[ADC_Operations +1];  		// get ADC val

extern int ADC_chN;
extern int ADC_RdN;

extern char ADC_flag;
extern char SetLedFlag;

extern uint16_t ADC_GlobFlag;
/* ---------------------------------------------------------------------------*/

#endif /* INC_ADC_H_ */
