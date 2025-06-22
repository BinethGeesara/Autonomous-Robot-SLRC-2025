#include "i2c_MUX.h"

// External I2C handle (ensure this is declared in your main code or globally)
extern I2C_HandleTypeDef hi2c1;

void setTCAChannel(uint8_t i) {
    uint8_t channel = 1 << i;
    HAL_I2C_Master_Transmit(&hi2c1, (0x70 << 1), &channel, 1, 1000);
}