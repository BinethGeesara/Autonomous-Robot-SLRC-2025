#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "u8g2.h"

// OLED display instance
extern u8g2_t u8g2;

// Function prototypes
void OLED_Init(void);
void OLED_Name(void);
void OLED_DisplayText(const char *text, uint8_t x, uint8_t y);
void OLED_DrawBox(uint8_t x, uint8_t y, uint8_t width, uint8_t height);
void OLED_Update(void);
void TASK_1_OLED(void);
void TASK_6_OLED(void);
void caliberate_OLED(void);
void display_TASK();
void RED();
void BLUE();
void alldone(void);
#endif // OLED_DISPLAY_H