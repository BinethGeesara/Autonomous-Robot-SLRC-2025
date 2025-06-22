/*
 * pid.c
 */

#include "main.h"
#include "motor_control.h"
#include "encoders.h"

/* Declaration of global variables used in this file */

/* distance constants, goal, threshold & errors: */
float kPx = 0.4;
float kDx = 0;
float goalDistance = 0;
int distanceThreshold = 2; /* This variable determines the range (in encoder counts) at which the move is finished. For example if this variable if 5, the mouse will consider the goal reached if the distance error is in the range -5 <= distanceError <= 5 */
float previousDistanceErrors[6] = {0, 0, 0, 0, 0, 0};		/* stores previous error terms to average them for more accurate derivative correction */

/* angle constants, goal, threshold & errors: */
float kPw = 0.7;
float kDw = 0;
float goalAngle = 0;
int angleThreshold = 2; /* This variable determines the range at which the turn is finished. For example if this variable if 5, the mouse will consider the goal reached if the angle error is in the range -5 <= angleError <= 5 */
float previousAngleErrors[6] = {0, 0, 0, 0, 0, 0};			/* stores previous error terms to average them for more accurate derivative correction */
int systickCalls ;
/* This variable stores the minimum pwm at which your mouse moves (experimentally determined). It's needed at times when the distance/angle correction set the motor pwms to a value lower than one at which the mouse can move at. This also depends on battery voltage, but a rough estimate should be fine */
float minimumSpeed = 0.2;


void resetPID() {
	/* resetting global variables */
	for (int i = 0; i < 6; ++i) {
		previousDistanceErrors[i] = 0;
		previousAngleErrors[i] = 0;
	}
	goalDistance = 0;
	goalAngle = 0;
	systickCalls = 0;
	
	/* resetting the motors & encoders */
	resetEncoders();
	setMotorSpeedL(0);
	setMotorSpeedR(0);
}

void updatePID() {	
	/* Updating the distance errors & calculating the distance correction */
	
	int distanceError = goalDistance - (getRightEncoderCounts() + getLeftEncoderCounts()) / 2;
	/* finds the average of the past 5 error values & updates errors array: */
	int avgDistanceErrors = 0;
	for (int i = 0; i < 5; ++i) {
		previousDistanceErrors[i] = previousDistanceErrors[i + 1];
		avgDistanceErrors += previousDistanceErrors[i];
	}
	previousDistanceErrors[5] = distanceError;
	avgDistanceErrors = (avgDistanceErrors + distanceError) / 5;
	int distanceCorrection = kPx * distanceError + kDx * (distanceError - avgDistanceErrors);
	
	/* Updating the angle errors & calculating the angle correction */
	
	int angleError = goalAngle - (getLeftEncoderCounts() - getRightEncoderCounts());
	/* finds the average of the past 5 error values & updates errors array: */
	int avgAngleErrors = 0;
	for (int i = 0; i < 5; ++i) {
		previousAngleErrors[i] = previousAngleErrors[i + 1];
		avgAngleErrors += previousAngleErrors[i];
	}
	previousAngleErrors[5] = angleError;
	avgAngleErrors = (avgAngleErrors + angleError) / 5;
	int angleCorrection = kPw * angleError + kDw * (angleError - avgAngleErrors);
	
	
	/* Update motor pwm values while accounting for minimum speed */
	
	int rightPWM = distanceCorrection - angleCorrection;
	int leftPWM = distanceCorrection + angleCorrection;
	
	if (rightPWM > -minimumSpeed && rightPWM < minimumSpeed) {
		if (rightPWM > 0)
			rightPWM = minimumSpeed;
		else
			rightPWM = -minimumSpeed;
	}
	
	if (leftPWM > -minimumSpeed && leftPWM < minimumSpeed) {
		if (leftPWM > 0)
			leftPWM = minimumSpeed;
		else
			leftPWM = -minimumSpeed;
	}
	
	setMotorSpeedL(rightPWM);
	setMotorSpeedR(leftPWM);
	
	
	/* checks if it the current encoder counts are within the goal threshold, if so, increments the systickCalls variable */
	if (distanceError >= -distanceThreshold && distanceError <= distanceThreshold && angleError >= -angleThreshold && angleError <= angleThreshold)
		systickCalls++;
	else 
		systickCalls = 0;
}

void setPIDGoalD(int16_t distance) {
	goalDistance = distance;
}

void setPIDGoalA(int16_t angle) {
	goalAngle = angle;
}

int8_t PIDdone(void) { // There is no bool type in C. True/False values are represented as 1 or 0.
	if (systickCalls >= 50)
		return 1;
	else 
		return 0;
}
