#include "tcs34725.h"
#include "Timer_Delay.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t _tcs34725Initialised = 0;
int red, green, blue;
int red1, green1, blue1;

float R, G, B;
ColorType topColor, boxColor, bottomColor;

/* Writes a register and an 8 bit value over I2C */
void write8(uint8_t reg, uint32_t value)
{
    uint8_t txBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    txBuffer[1] = (value & 0xFF);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, txBuffer, 2, 100);
}

/* Reads an 8 bit value over I2C */
uint8_t read8(uint8_t reg)
{
    uint8_t buffer[1];
    buffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, buffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, buffer, 1, 100);
    return buffer[0];
}

/* Reads a 16 bit values over I2C */
uint16_t read16(uint8_t reg)
{
    uint16_t ret;
    uint8_t txBuffer[1], rxBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, txBuffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, rxBuffer, 2, 100);
    ret = rxBuffer[1];
    ret <<= 8;
    ret |= rxBuffer[0] & 0xFF;
    return ret;
}

void enable(void)
{
    write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
    delay_ms(3);
    write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    delay_ms(50);
}

void disable(void)
{
    /* Turn the device off to save power */
    uint8_t reg = 0;
    reg = read8(TCS34725_ENABLE);
    write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

void setIntegrationTime(uint8_t itime)
{
    if (_tcs34725Initialised == 0)
        tcs3272_init();
    write8(TCS34725_ATIME, itime);
}

void setGain(uint8_t gain)
{
    if (_tcs34725Initialised == 0)
        tcs3272_init();
    write8(TCS34725_CONTROL, gain);
}

void tcs3272_init(void)
{
    /* Make sure we're actually connected */
    uint8_t readValue = read8(TCS34725_ID);

    _tcs34725Initialised = 1;
    /* Set default integration time and gain */
    setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
    setGain(TCS34725_GAIN_4X);
    /* Note: by default, the device is in power down mode on bootup */
    enable();
}

/* Get raw data */
void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    if (_tcs34725Initialised == 0)
        tcs3272_init();

    *c = read16(TCS34725_CDATAL);
    *r = read16(TCS34725_RDATAL);
    *g = read16(TCS34725_GDATAL);
    *b = read16(TCS34725_BDATAL);
    /* Delay time is from page no 16/26 from the datasheet  (256 − ATIME)* 2.4ms */
    delay_ms(50); // Set delay for (256 − 0xEB)* 2.4ms = 50ms
}

/* Get Red, Green and Blue color from Raw Data */
void normalizeRGB(float *R, float *G, float *B, uint16_t rawRed, uint16_t rawGreen, uint16_t rawBlue, uint16_t rawClear)
{
    if (rawClear == 0)
    {
        *R = 0;
        *G = 0;
        *B = 0;
    }
    else
    {
        *R = (float)rawRed / rawClear;
        *G = (float)rawGreen / rawClear;
        *B = (float)rawBlue / rawClear;
    }
}
ColorType identify_Ball_O_W(float R, float G, float B)
{
    // Determine the dominant color
    if (R > B)
    {
        return COLOR_ORANGE;
    }
    else if (B > R || R == B)
    {
        return COLOR_WHITE; // Green is dominant
    }
    else
    {
        // If no clear dominant color, return unknown
        return COLOR_UNKNOWN;
    }
}
ColorType identify_BOXES(float R, float G, float B)
{
    // Determine the dominant color
    if (R > G && R > B)
    {
        return COLOR_RED; // Red is dominant
    }
    else 
    {
        return COLOR_BLUE; // Blue is dominant
    }
    
}

ColorType identify_PADS(float R, float G, float B)
{
    // Determine the dominant color
    if (R > G && R > B)
    {
        return COLOR_RED; // Red is dominant
    }
    else if (G > 0.45)
    {
        return COLOR_GREEN; // Green is dominant
    }
    else if (B > R && B > G)
    {
        return COLOR_BLUE; // Blue is dominant
    }
    else
    {
        // If no clear dominant color, return unknown
        return COLOR_UNKNOWN;
    }
}

void Top_C_BALL_Sensor()
{
    uint16_t rawRed, rawGreen, rawBlue, rawClear;
    getRawData(&rawRed, &rawGreen, &rawBlue, &rawClear);

    normalizeRGB(&R, &G, &B, rawRed, rawGreen, rawBlue, rawClear);
    topColor = identify_Ball_O_W(R, G, B);
    // Handle the identified color (e.g., print or store it)
}
void Top_C_BOX_Sensor()
{
    uint16_t rawRed, rawGreen, rawBlue, rawClear;
    getRawData(&rawRed, &rawGreen, &rawBlue, &rawClear);

    normalizeRGB(&R, &G, &B, rawRed, rawGreen, rawBlue, rawClear);
    boxColor = identify_BOXES(R, G, B);
    // Handle the identified color (e.g., print or store it)
}

void Bottom_C_PAD_Sensor()
{
    uint16_t rawRed, rawGreen, rawBlue, rawClear;
    getRawData(&rawRed, &rawGreen, &rawBlue, &rawClear);

    normalizeRGB(&R, &G, &B, rawRed, rawGreen, rawBlue, rawClear);
    bottomColor = identify_PADS(R, G, B);
    // Handle the identified color (e.g., print or store it)
}
/*0: COLOR_UNKNOWN

1: COLOR_WHITE

2: COLOR_ORANGE

3: COLOR_RED

4: COLOR_GREEN

5: COLOR_BLUE*/
