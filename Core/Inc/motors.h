/**
 * Motor driving abstractions.
 */

#pragma once

#include "stm32f4xx_hal.h"

/*
 * Default motor scale limits.
 */
#define MOTOR_SCALE_MAX_FWD 1
#define MOTOR_SCALE_MAX_BACK -1

/**
 * Represents a timer PWM driver.
 */
typedef struct
{
	/**
	 * ST TIM handler.
	 */
	TIM_HandleTypeDef *timer;

	/**
	 * The timer channel to output on.
	 */
	uint32_t channel;

	/**
	 * The timer period.
	 */
	uint32_t period;
} PWM_Driver;

/**
 * Represents a motor.
 */
typedef struct
{
	/**
	 * The timer PWM for forwards driving.
	 */
	PWM_Driver fwd;

	/**
	 * The timer PWM for backwards driving.
	 */
	PWM_Driver back;

	/**
	 * The maximum forward driving power scale.
	 */
	float max_fwd;

	/**
	 * The maximum reverse driving power scale.
	 */
	float max_back;
} Motor;

/**
 * Initialize the parameters of a motor struct to default
 * values.
 *
 * @param[in] motor pointer to motor object to initialize.
 * @param[in] fw_t the timer handler for forward driving of the motor.
 * @param[in] fw_channel the timer channel used for forward driving.
 * @param[in] bk_t the timer channel for reverse driving of the motor.
 * @param[in] bk_channel the timer channel used for reverse driving.
 */
void init_default_motor(Motor *motor,
						TIM_HandleTypeDef *fw_t,
						uint32_t fw_channel,
						TIM_HandleTypeDef *bk_t,
						uint32_t bk_channel)
{
	/*
	 * Forward driving PWM.
	 */
	PWM_Driver fw = {
			fw_t,
			fw_channel,
			fw_t->Init.Period
	};

	/*
	 * Reverse driving PWM.
	 */
	PWM_Driver bk = {
			bk_t,
			bk_channel,
			bk_t->Init.Period
	};

	/*
	 * Fill parameters.
	 */
	motor->fwd = fw;
	motor->back = bk;
	motor->max_fwd = MOTOR_SCALE_MAX_FWD;
	motor->max_back = MOTOR_SCALE_MAX_BACK;
}

/**
 * Drive the PWM of the motor.
 *
 * @param[in] motor a pointer to the motor object to drive.
 * @param[in] power the duty cycle and direction to drive.
 *
 */
void drive_motor(Motor *motor, float power)
{


	PWM_Driver dir;
	PWM_Driver reverse;

	if (power > 0)
	{
		dir = motor->fwd;
		reverse = motor->back;
		power = (power > motor->max_fwd) ? motor->max_fwd : power;
	}
	else
	{
		dir = motor->back;
		reverse = motor->fwd;
		power = (power < motor->max_back) ? -motor->max_back : -power;
	}

	/*
	 * Clear timer for driving on opposite direction.
	 */
	__HAL_TIM_SET_COMPARE(
			reverse.timer,
			reverse.channel,
			0
	);

	/*
	 * Set timer for driving in the desired direction.
	 */
	__HAL_TIM_SET_COMPARE(
			dir.timer,
			dir.channel,
			dir.period * power
	);
}
