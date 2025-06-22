/*
 * encoders.c
 */

#include "main.h"
#include "encoders.h"
#include "controller.h"

int16_t right_encoder;
int16_t left_encoder;
/*
 * Implement this function so it returns the right encoder value
 */
int16_t getRightEncoderCounts() {
	right_encoder = (int16_t) TIM2->CNT;
	return (int16_t) TIM2->CNT;
}

/*
 * Implement this function so it returns the left encoder value
 */
int16_t getLeftEncoderCounts() {
	left_encoder = (int16_t) TIM3->CNT;
	return (int16_t) TIM3->CNT;
}

/*
 * This function has already been implemented for you. Enjoy! :)
 */
void resetEncoders() {
	TIM2->CNT = (int16_t) 0;
	TIM3->CNT = (int16_t) 0;
}
