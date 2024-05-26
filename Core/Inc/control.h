#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "stm32f4xx_hal.h"
#include "motors.h"
#include "distance.h"



// 1. CONSTANTS

// Interrupt time constants
static const double TIME_STEP = 0.001; // time step of periodic interrupt (see control.h) in seconds (current: 1ms)
static const uint16_t ENCODER_READ_FREQUENCY = 7; // How often to read encoder data (How many TIME_STEPs between readings)

// Smoothing values for sensors (range 0-1)
static const double SMOOTHING_VALUE_DISTANCE = 0.2; // these are backwards. Closer to 0 is more smoothing. 1 is no smoothing
static const double SMOOTHING_VALUE_DISTANCE_DERIVATIVE = 0.05;
static const double SMOOTHING_VALUE_VELOCITY = 0.2;

// PID Terms velocity
static const double KP_VELOCITY = 0.002;
static const double KI_VELOCITY = 0.0001;
static const double KD_VELOCITY = 0.002;

// Trapezoidal velocity profile
static const double ACCELERATION_DISTANCE = 200; // in mm
static const double TARGET_DISTANCE_THRESHOLD = 1; // in mm



// 2. PRIVATE VARIABLES

TIM_HandleTypeDef timerPeriodic;

static uint32_t interruptStartCounter = 0;
static uint32_t interruptEndCounter = 0;

static SensorDistances_t distanceSmoothed;
static SensorDistances_t distanceSmoothedPrev;
static SensorDistances_t distanceDerivativeSmoothed;

static EncoderCount_t encoderCountPrev;
static EncoderCount_t encoderDelta;
static EncoderCount_t encoderDeltaPrev;

static MotorVector_t motorVelocitySmoothed;

static MotorVector_t velocitySetpoint = {100, 100};
static MotorVector_t setpointErrorPrev;

static EncoderCount_t encoderCountsTraveled = {0, 0};
static double targetDistance = 141.3716694; //90*MM_PER_DEGREE;

uint16_t PIDLogicCounter = 0;


void ControlInit(TIM_HandleTypeDef *_timerPeriodic) {
	// Store timer handle
	timerPeriodic = *_timerPeriodic;

	// Start periodic interrupt
	HAL_TIM_Base_Start_IT(&timerPeriodic);
}

void TrapezoidalDistance() {
	// Trapezoidal velocity profile

	encoderCountsTraveled.left += encoderDelta.left;
	encoderCountsTraveled.right += encoderDelta.right;
	MotorVector_t distanceTraveled = EncoderCountToMillimeters(encoderCountsTraveled);
	double averageDistance = 0.5*(distanceTraveled.left + distanceTraveled.right);
	double accelerationDistance = ACCELERATION_DISTANCE;
	if (0.5*targetDistance < accelerationDistance)
		accelerationDistance = 0.5*targetDistance;

	if (averageDistance <= targetDistance+TARGET_DISTANCE_THRESHOLD && averageDistance >= targetDistance-TARGET_DISTANCE_THRESHOLD) {
		velocitySetpoint.left = 0;
		velocitySetpoint.right = 0;
	} else if (targetDistance - averageDistance <= accelerationDistance && targetDistance - averageDistance > 0) {
		double v = (targetDistance - averageDistance)/ACCELERATION_DISTANCE*MAX_SPEED;
		if (v < MIN_SPEED)
			v = MIN_SPEED;
		velocitySetpoint.left = v;
		velocitySetpoint.right = v;
	} else if(averageDistance <= accelerationDistance) {
		double v = averageDistance/ACCELERATION_DISTANCE*MAX_SPEED;
		if (v < MIN_SPEED)
			v = MIN_SPEED;
		velocitySetpoint.left = v;
		velocitySetpoint.right = v;
	} else if (averageDistance > targetDistance+TARGET_DISTANCE_THRESHOLD) {
		velocitySetpoint.left = -MIN_SPEED;
		velocitySetpoint.right = -MIN_SPEED;
	}
}

void TrapezoidalTurn() {
	// Trapezoidal velocity profile

	encoderCountsTraveled.left += encoderDelta.left;
	encoderCountsTraveled.right += encoderDelta.right;
	MotorVector_t distanceTraveled = EncoderCountToMillimeters(encoderCountsTraveled);
	double averageDistance = 0.5*(distanceTraveled.left - distanceTraveled.right);
	double accelerationDistance = ACCELERATION_DISTANCE;
	if (0.5*targetDistance < accelerationDistance)
		accelerationDistance = 0.5*targetDistance;

	if (averageDistance <= targetDistance+TARGET_DISTANCE_THRESHOLD && averageDistance >= targetDistance-TARGET_DISTANCE_THRESHOLD) {
		velocitySetpoint.left = 0;
		velocitySetpoint.right = 0;
	} else if (targetDistance - averageDistance <= accelerationDistance && targetDistance - averageDistance > 0) {
		double v = (targetDistance - averageDistance)/ACCELERATION_DISTANCE*MAX_SPEED;
		if (v < MIN_SPEED)
			v = MIN_SPEED;
		velocitySetpoint.left = v;
		velocitySetpoint.right = -v;
	} else if(averageDistance <= accelerationDistance) {
		double v = averageDistance/ACCELERATION_DISTANCE*MAX_SPEED;
		if (v < MIN_SPEED)
			v = MIN_SPEED;
		velocitySetpoint.left = v;
		velocitySetpoint.right = -v;
	} else if (averageDistance > targetDistance+TARGET_DISTANCE_THRESHOLD) {
		velocitySetpoint.left = -MIN_SPEED;
		velocitySetpoint.right = MIN_SPEED;
	}
}

// Periodic timer interrupt for main logic loop
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	interruptStartCounter++;

	// Read sensor distances and smooth
	SensorDistances_t dist = ReadDistances();
	distanceSmoothed.left = dist.left*SMOOTHING_VALUE_DISTANCE + distanceSmoothed.left*(1-SMOOTHING_VALUE_DISTANCE);
	distanceSmoothed.middle = dist.middle*SMOOTHING_VALUE_DISTANCE + distanceSmoothed.middle*(1-SMOOTHING_VALUE_DISTANCE);
	distanceSmoothed.right = dist.right*SMOOTHING_VALUE_DISTANCE + distanceSmoothed.right*(1-SMOOTHING_VALUE_DISTANCE);

	SensorDistances_t distanceDerivative;
	distanceDerivative.left = (distanceSmoothed.left - distanceSmoothedPrev.left)/TIME_STEP;
	distanceDerivative.middle = (distanceSmoothed.middle - distanceSmoothedPrev.middle)/TIME_STEP;
	distanceDerivative.right = (distanceSmoothed.right - distanceSmoothedPrev.right)/TIME_STEP;

	distanceDerivativeSmoothed.left = distanceDerivative.left*SMOOTHING_VALUE_DISTANCE_DERIVATIVE + distanceDerivativeSmoothed.left*(1-SMOOTHING_VALUE_DISTANCE_DERIVATIVE);
	distanceDerivativeSmoothed.middle = distanceDerivative.middle*SMOOTHING_VALUE_DISTANCE_DERIVATIVE + distanceDerivativeSmoothed.middle*(1-SMOOTHING_VALUE_DISTANCE_DERIVATIVE);
	distanceDerivativeSmoothed.right = distanceDerivative.right*SMOOTHING_VALUE_DISTANCE_DERIVATIVE + distanceDerivativeSmoothed.right*(1-SMOOTHING_VALUE_DISTANCE_DERIVATIVE);

	distanceSmoothedPrev = distanceSmoothed;


	if (interruptStartCounter < 2000) { //interrupt timer
		interruptEndCounter++;
		return;
	}


	double averageSideDerivative;
	double averageSideDistance;
	const double MAX_DERIV = 60;
	// smooths data by removing noise
	if(distanceSmoothed.left <= 100 && fabs(distanceDerivativeSmoothed.left) < MAX_DERIV && distanceSmoothed.right <= 100 && fabs(distanceDerivativeSmoothed.right) < MAX_DERIV) {
		averageSideDerivative = 0.5*(distanceDerivativeSmoothed.left - distanceDerivativeSmoothed.right);
		averageSideDistance = distanceSmoothed.left - distanceSmoothed.right;
	}else if (distanceSmoothed.left <= 100 && fabs(distanceDerivativeSmoothed.left) < MAX_DERIV) {
		averageSideDerivative = distanceDerivativeSmoothed.left;
		averageSideDistance = distanceSmoothed.left - 90;
	} else if (distanceSmoothed.right <= 100 && fabs(distanceDerivativeSmoothed.right) < MAX_DERIV) {
		averageSideDerivative = -distanceDerivativeSmoothed.right;
		averageSideDistance = 90 - distanceSmoothed.right;
	} else {
		averageSideDerivative = 0;
		averageSideDistance = 0;
	}
	averageSideDistance *= 0.02;
	averageSideDerivative *= 0.5;

	// Encoder reading and PID for motor PWM
	if (++PIDLogicCounter >= ENCODER_READ_FREQUENCY) {
		PIDLogicCounter = 0;


		// Read and process encoder values

		// Read encoder values, get delta since last periodic interrupt
		EncoderCount_t encoderCount = ReadEncoders();
		encoderDelta.left = encoderCount.left - encoderCountPrev.left;
		encoderDelta.right = encoderCount.right - encoderCountPrev.right;
		encoderCountPrev = encoderCount;
		// Check if encoder counter over/underflowed and use previous value if so
		if (encoderDelta.right > 32768) // arbitrary large number. Like over 7 m/s or so
			encoderDelta.right = encoderDeltaPrev.right;
		if (encoderDelta.left > 32768)
			encoderDelta.left = encoderDeltaPrev.left;
		// Record encoder delta
		encoderDeltaPrev = encoderDelta;
		// Convert encoder deltas to velocities (mm/s)
		MotorVector_t motorVelocity = EncoderCountToMillimeters(encoderDelta);
		motorVelocity.left /= TIME_STEP*ENCODER_READ_FREQUENCY;
		motorVelocity.right /= TIME_STEP*ENCODER_READ_FREQUENCY;
		// Smooth velocity readings
		motorVelocitySmoothed.left = motorVelocity.left*SMOOTHING_VALUE_VELOCITY + motorVelocitySmoothed.left*(1-SMOOTHING_VALUE_VELOCITY);
		motorVelocitySmoothed.right = motorVelocity.right*SMOOTHING_VALUE_VELOCITY + motorVelocitySmoothed.right*(1-SMOOTHING_VALUE_VELOCITY);


		// Motor velocity PID

		// Calculate setpoint error, derivative, integral
		MotorVector_t setpointError;
		setpointError.left = velocitySetpoint.left - motorVelocitySmoothed.left;// - averageSideDerivative - averageSideDistance;
		setpointError.right = velocitySetpoint.right - motorVelocitySmoothed.right;// + averageSideDerivative + averageSideDistance;
		MotorVector_t setpointDelta;
		setpointDelta.left = setpointError.left - setpointErrorPrev.left;
		setpointDelta.right = setpointError.right - setpointErrorPrev.right;
		setpointErrorPrev = setpointError;

		// Calculate PID terms
		MotorVector_t proportionalTerm;
		proportionalTerm.left = setpointError.left*KP_VELOCITY;
		proportionalTerm.right = setpointError.right*KP_VELOCITY;
		MotorVector_t derivativeTerm;
		derivativeTerm.left = setpointDelta.left*KD_VELOCITY;
		derivativeTerm.right = setpointDelta.right*KD_VELOCITY;
		static MotorVector_t integralTerm;
		integralTerm.left += setpointError.left*KI_VELOCITY;
		integralTerm.right += setpointError.right*KI_VELOCITY;

		// Calculate PID error sum
		MotorVector_t errorSum;
		errorSum.left = proportionalTerm.left + derivativeTerm.left + integralTerm.left;
		errorSum.right = proportionalTerm.right + derivativeTerm.right + integralTerm.right;

		// Update motor speeds
		SetMotorSpeedLeft(errorSum.left);
		SetMotorSpeedRight(errorSum.right);


		//TrapezoidalDistance();
		TrapezoidalTurn();
	}


	interruptEndCounter++;
}

#endif /* INC_CONTROL_H_ */
