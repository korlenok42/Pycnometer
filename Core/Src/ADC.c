/*
 * ADC.c
 *
 *  Created on: 13 нояб. 2022 г.
 *      Author: Irina
 */

#include "ADC.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int ADC_d12 = 0;

uint16_t ADC_GlobFlag  = 0;
char ADC_flag = 0;
char SetLedFlag = 0;

int ADC_chN = 0; // [0..16]
int ADC_RdN = 0; // [0..15]

//
int ADC_Buf_j = 0; // [0..3]
//

int ADC_Buf[ADC_Operations + 1];  		// get ADC val
//======================================================================
// ADC average value calculation
void ADC_AverageMean (void)
{

	// ADC_chN - global value [0..17]
	// ADC_Buf_j - global value, [0..3]

	ADC_BufMean = 0;

	for (int i = 0; i <= ADC_Operations; i++)
	{
		ADC_BufMean = ADC_BufMean + ADC_Buf[i];
	} // for i

	ADC_BufMean = ADC_BufMean >> 4;
}
//======================================================================

/*uint16_t FinalNumber (uint16_t counts)
{
	uint16_t start, final, result;

	for(int i = 0; i <= ADC_Operations; i++)
	{
		// +- 10 counts
		start = i*270;
		final = ((i+1)*270) + 40;

		if (counts >= start && counts <= final)
			result = i;
	}

	return result;
}
*/

void CheckNumber (uint16_t counts)
{
	uint16_t start, final;
	uint16_t result = 0;

	for(int i = 0; i <= ADC_Operations; i++)
	{
		// +- 10 counts
		start = i*270 + 30;
		final = ((i+1)*270);

		if ( (counts >= start && counts <= final) && i != GlobalTempValue)
		{
			SetLedFlag = 1;
			GlobalTempValue = i;
		}
	}

}


