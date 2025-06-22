#include "motor_control.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

#define MAX_TIMER_COUNTS 3817
#define MAX_PWM_VALUE  255    // Full speed range (-255 to +255)

void setMotorSpeedR(int pwm) {
    pwm = limitPWM(pwm);  // Ensure PWM is within range

    if (pwm > 0) {
        // Ensure the motor runs by setting a minimum working PWM
        uint32_t dutyCycle = (pwm < MIN_WORKING_PWM) ? MIN_WORKING_PWM : pwm;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dutyCycle * MAX_TIMER_COUNTS / MAX_PWM_VALUE));
    }
    else if (pwm < 0) {
        // Ensure the motor runs by setting a minimum working PWM
        uint32_t dutyCycle = (-pwm < MIN_WORKING_PWM) ? MIN_WORKING_PWM : -pwm;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); 
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)(dutyCycle * MAX_TIMER_COUNTS / MAX_PWM_VALUE));
    }
    else {
        // If pwm == 0, stop the motor completely
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}

void setMotorSpeedL(int pwm) {
    pwm = limitPWM(pwm);  // Ensure PWM is within range

    if (pwm > 0) {
        // Ensure the motor runs by setting a minimum working PWM
        uint32_t dutyCycle = (pwm < MIN_WORKING_PWM) ? MIN_WORKING_PWM : pwm;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);  // Stop reverse
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dutyCycle * MAX_TIMER_COUNTS / MAX_PWM_VALUE));
    }
    else if (pwm < 0) {
        // Ensure the motor runs by setting a minimum working PWM
        uint32_t dutyCycle = (-pwm < MIN_WORKING_PWM) ? MIN_WORKING_PWM : -pwm;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);  // Stop forward
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(dutyCycle * MAX_TIMER_COUNTS / MAX_PWM_VALUE));
    }
    else {
        // If pwm == 0, stop the motor completely
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
}
int limitPWM(int pwm) {
    if (pwm > MAX_PWM_VALUE) return MAX_PWM_VALUE;
    if (pwm < -MAX_PWM_VALUE) return -MAX_PWM_VALUE;
    return pwm;
}
void m_stopLR() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}
void m_stopL() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}
void m_stopR() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}
