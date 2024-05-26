#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include <stdlib.h>
#include "stm32f4xx_hal.h"
//const double MM_PER_ENCODER_COUNT =0.11050759;
const double CM_PER_ENCODER_COUNT = 0.07942733;
//const double MM_PER_DEGREE = 0.76794487;
//const double CM_PER_DEGREE = 0.157079633;
const double MM_PER_DEGREE = 1.570796327;
const double MAX_SPEED = 400; // mm/s
const double MIN_SPEED = 20;
//const double MAX_SPEED = 40;
//const double MIN_SPEED = 2; //cm/s

typedef struct {
	int32_t left;
	int32_t right;
} EncoderCount_t;

typedef struct {
	double left;
	double right;
} MotorVector_t;


static TIM_HandleTypeDef timerLeftMotor;
static TIM_HandleTypeDef timerLeftEncoder;
static TIM_HandleTypeDef timerRightMotor;
static TIM_HandleTypeDef timerRightEncoder;

// handles speed and direction
void MotorInit(TIM_HandleTypeDef *_timerLeftMotor, TIM_HandleTypeDef *_timerLeftEncoder, TIM_HandleTypeDef *_timerRightMotor, TIM_HandleTypeDef *_timerRightEncoder) {
	// Store timer handles
	timerLeftMotor = *_timerLeftMotor;
	timerLeftEncoder = *_timerLeftEncoder;
	timerRightMotor = *_timerRightMotor;
	timerRightEncoder = *_timerRightEncoder;

	// Start encoder timers
	HAL_TIM_Encoder_Start(&timerLeftEncoder, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&timerRightEncoder, TIM_CHANNEL_ALL);

	// Start left motor timers
	__HAL_TIM_SET_COMPARE(&timerLeftMotor, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&timerLeftMotor, TIM_CHANNEL_2, 0);
	HAL_TIM_PWM_Start(&timerLeftMotor, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&timerLeftMotor, TIM_CHANNEL_2);

	// Start right motor timers
	__HAL_TIM_SET_COMPARE(&timerRightMotor, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&timerRightMotor, TIM_CHANNEL_2, 0);
	HAL_TIM_PWM_Start(&timerRightMotor, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&timerRightMotor, TIM_CHANNEL_2);
}


EncoderCount_t ReadEncoders() {
	EncoderCount_t encoderCounts;
	encoderCounts.left = __HAL_TIM_GET_COUNTER(&timerLeftEncoder);
	encoderCounts.right = __HAL_TIM_GET_COUNTER(&timerRightEncoder);
	return encoderCounts;
}

MotorVector_t EncoderCountToMillimeters(EncoderCount_t _counts) {
	MotorVector_t vec;
	vec.left = _counts.left*CM_PER_ENCODER_COUNT;
	vec.right = _counts.right*CM_PER_ENCODER_COUNT;
	return vec;
}


// `_speed` should be in the range -1 to 1
void SetMotorSpeedLeft(double _speed) {
	// Clamp speed within range -1 to 1
	if (fabs(_speed) > 1)
		_speed /= fabs(_speed);

	if (_speed < 0) {
		_speed *= -1;
		__HAL_TIM_SET_COMPARE(&timerLeftMotor, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&timerLeftMotor, TIM_CHANNEL_2, _speed*timerLeftMotor.Init.Period-1);
	} else {
		__HAL_TIM_SET_COMPARE(&timerLeftMotor, TIM_CHANNEL_1, _speed*timerLeftMotor.Init.Period-1);
		__HAL_TIM_SET_COMPARE(&timerLeftMotor, TIM_CHANNEL_2, 0);
	}
}


// `_speed` should be in the range -1 to 1
void SetMotorSpeedRight(double _speed) {
	// Clamp speed within range -1 to 1
	if (abs(_speed) > 1)
		_speed /= abs(_speed);

	if (_speed < 0) {
		_speed *= -1;
		__HAL_TIM_SET_COMPARE(&timerRightMotor, TIM_CHANNEL_1, _speed*timerRightMotor.Init.Period-1);
		__HAL_TIM_SET_COMPARE(&timerRightMotor, TIM_CHANNEL_2, 0);
	} else {
		__HAL_TIM_SET_COMPARE(&timerRightMotor, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&timerRightMotor, TIM_CHANNEL_2, _speed*timerRightMotor.Init.Period-1);
	}
}


#endif /* INC_MOTORS_H_ */
