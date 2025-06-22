#ifndef PCA9685_H
#define PCA9685_H

#include "main.h"

// PCA9685 I2C Address
#define PCA9685_ADDRESS 0x80

// PCA9685 Registers
#define PCA9685_MODE1         0x0
#define PCA9685_PRE_SCALE     0xFE
#define PCA9685_LED0_ON_L     0x6

// PCA9685 Mode1 Bits
#define PCA9685_MODE1_SLEEP_BIT      4
#define PCA9685_MODE1_AI_BIT         5
#define PCA9685_MODE1_RESTART_BIT    7

// Function Prototypes
void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value);
void PCA9685_SetPWMFrequency(uint16_t frequency);
void PCA9685_Init(uint16_t frequency);
void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
void PCA9685_SetServoAngle(uint8_t Channel, float Angle);
void PCA9685_SetServoAngleSlow(uint8_t Channel, float StartAngle, float EndAngle, uint16_t DelayMs);

#endif // PCA9685_H