#ifndef INC_DISTANCE_H_
#define INC_DISTANCE_H_

#include "stm32f4xx_hal.h"

typedef struct {
	uint32_t left;
	uint32_t middle;
	uint32_t right;
} ADCValues_t;

typedef struct {
	double left;
	double middle;
	double right;
} SensorDistances_t;

ADC_HandleTypeDef adc;
DMA_HandleTypeDef dma;

static ADCValues_t rawADCValues;

const static uint16_t NUM_DISTANCES = 21;
static double distanceArray[21] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200};
static uint32_t ADCCountsLeft[21] = {2290, 1779,1128,741,532,403,322,268,234,207,186,162,149,143,134,129,122,117,112,108,104};
static uint32_t ADCCountsMiddle[21] = {2400, 2241,1405,882,642,435,376,306,268,238,214,195,181,172,164,154,149,144,140,136,134};
static uint32_t ADCCountsRight[21] = {1848,1000,696,495,393,320,268,230,194,186,161,147,139,130,126,120,115,110,106,102};

void DistanceInit(ADC_HandleTypeDef *_adc, DMA_HandleTypeDef *_dma){
	adc = *_adc;
	dma = *_dma;

	HAL_ADC_Start_DMA(&adc,(uint16_t*)&rawADCValues,3);
	ADC1->CR2 |= ADC_CR2_DDS;
}

// Converts ADC counts to a millimeter value using a lookup table and interpolating between values
double ADCCounttoMillimeters(uint32_t _ADCcounts, double _distanceArray[], uint32_t _countArray[], uint16_t _numDistances) {

	if (_ADCcounts < _countArray[_numDistances-1])
		return _distanceArray[_numDistances-1] + (_ADCcounts-_countArray[_numDistances-1])*(_distanceArray[_numDistances-1]-_distanceArray[_numDistances-2])/(_countArray[_numDistances-1]-_countArray[_numDistances-2]); // linear extrapolation

	// Find the index of first value in _countArray less than the ADC reading
	uint16_t i;
	uint32_t j;
	i = 0;
	j = (sizeof(_countArray)/ sizeof(uint32_t)) - 1;
	while(j > i){

		if (_ADCcounts > _countArray[i]|| _ADCcounts > _countArray[j]){
			break;
		}else{
			i++;
		j--;
	}
	}

	// Find the percentage of the way between the two nearest ADC count values in the _countArray
	double p = (double)(_ADCcounts - _countArray[i])/(double)(_countArray[i-1] - _countArray[i]);

	// Return linear interpolation between nearest two distances (in millimeters)
	return _distanceArray[i-1]*p + (1-p)*_distanceArray[i];
}


SensorDistances_t ReadDistances() {
	SensorDistances_t distances;
	distances.left = ADCCounttoMillimeters(rawADCValues.left, distanceArray, ADCCountsLeft, NUM_DISTANCES);
	distances.middle = ADCCounttoMillimeters(rawADCValues.middle, distanceArray, ADCCountsMiddle, NUM_DISTANCES);
	distances.right = ADCCounttoMillimeters(rawADCValues.right, distanceArray, ADCCountsRight, NUM_DISTANCES);
	return distances;
}

#endif /* INC_DISTANCE_H_ */
