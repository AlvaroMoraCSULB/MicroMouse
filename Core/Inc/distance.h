/*
 * distance.h
 *
 *  Created on: May 23, 2024
 *      Author: Andres_v
 */

#ifndef INC_DISTANCE_H_
#define INC_DISTANCE_H_

#include "stm32f4xx_hal.h"

static uint32_t ADCValue[3];

ADC_HandleTypeDef adc;
DMA_HandleTypeDef dma;

void DistanceInit(ADC_HandleTypeDef *_adc, DMA_HandleTypeDef *_dma){
	adc = *_adc;
	dma = *_dma;

	HAL_ADC_Start_DMA(&adc,(uint32_t*)&ADCValue,3);
	ADC1->CR2 |= ADC_CR2_DDS;
}



#endif /* INC_DISTANCE_H_ */
