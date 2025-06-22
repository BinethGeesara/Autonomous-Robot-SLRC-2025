#include "pca9685.h"

extern I2C_HandleTypeDef hi2c1; // Use the I2C handle defined in main.c

void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
    uint8_t readValue;
    HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
    if (Value == 0) readValue &= ~(1 << Bit);
    else readValue |= (1 << Bit);
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
    HAL_Delay(1);
}

void PCA9685_SetPWMFrequency(uint16_t frequency)
{
    uint8_t prescale;
    if (frequency >= 1526) prescale = 0x03;
    else if (frequency <= 24) prescale = 0xFF;
    else prescale = 25000000 / (4096 * frequency);
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRE_SCALE, 1, &prescale, 1, 10);
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

void PCA9685_Init(uint16_t frequency)
{
    PCA9685_SetPWMFrequency(frequency);
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}

void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
    uint8_t registerAddress = PCA9685_LED0_ON_L + (4 * Channel);
    uint8_t pwm[4] = {
        OnTime & 0xFF,
        OnTime >> 8,
        OffTime & 0xFF,
        OffTime >> 8
    };
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, registerAddress, 1, pwm, 4, 10);
}

void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
    float Value = (Angle * (511.9 - 102.4) / 180.0) + 102.4;
    PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}

void PCA9685_SetServoAngleSlow(uint8_t Channel, float StartAngle, float EndAngle, uint16_t DelayMs)
{
    float AngleIncrement = (EndAngle > StartAngle) ? 1.0 : -1.0;
    float CurrentAngle = StartAngle;

    while ((AngleIncrement > 0 && CurrentAngle <= EndAngle) || (AngleIncrement < 0 && CurrentAngle >= EndAngle))
    {
        PCA9685_SetServoAngle(Channel, CurrentAngle);
        CurrentAngle += AngleIncrement;
        HAL_Delay(DelayMs);
    }
}