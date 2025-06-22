#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#define MIN_WORKING_PWM 20
void setMotorSpeedR(int pwm);
void setMotorSpeedL(int pwm);
int limitPWM(int pwm);
void m_stopLR();
void m_stopL();
void m_stopR();
#endif // MOTOR_CONTROL_H
