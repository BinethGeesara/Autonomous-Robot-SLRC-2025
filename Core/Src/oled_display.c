#include "oled_display.h"
#include "main.h"

extern int Button1;
extern int Button2;
int start_caliberate = 0;
int start_task_1 = 0;
int start_task_2 = 0;
int start_task_3 = 0;
int start_task_4 = 0;
int start_task_5 = 0;
int start_task_6 = 0;

extern I2C_HandleTypeDef hi2c1;
// OLED display instance
u8g2_t u8g2;
// GPIO and delay callback for u8g2
uint8_t u8x8_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
    switch (msg)
    {
    case U8X8_MSG_DELAY_MILLI:
        HAL_Delay(arg_int);
        break;
    default:
        break;
    }
    return 1;
}
// I2C communication callback for u8g2
uint8_t u8x8_byte_hw_i2c_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static uint8_t buffer[32];
    static uint8_t buf_idx;
    uint8_t *data;

    switch (msg)
    {
    case U8X8_MSG_BYTE_SEND:
        data = (uint8_t *)arg_ptr;
        while (arg_int > 0)
        {
            buffer[buf_idx++] = *data;
            data++;
            arg_int--;
        }
        break;
    case U8X8_MSG_BYTE_INIT:
        break;
    case U8X8_MSG_BYTE_SET_DC:
        break;
    case U8X8_MSG_BYTE_START_TRANSFER:
        buf_idx = 0;
        break;
    case U8X8_MSG_BYTE_END_TRANSFER:
        HAL_I2C_Master_Transmit(&hi2c1, (0x3C << 1), buffer, buf_idx, HAL_MAX_DELAY);
        break;
    default:
        return 0;
    }
    return 1;
}
u8g2_uint_t ClampValue(int16_t value, u8g2_uint_t min, u8g2_uint_t max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return (u8g2_uint_t)value;
}
uint8_t MapValue(int16_t value, int16_t in_min, int16_t in_max, uint8_t out_min, uint8_t out_max)
{
    return out_min + (uint8_t)((float)(value - in_min) * (float)(out_max - out_min) / (float)(in_max - in_min));
}
// Initialize the OLED display
void OLED_Init(void)
{
    setTCAChannel(4);
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c_stm32, u8x8_gpio_and_delay_stm32);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
}
// Clear the OLED display
void OLED_Name(void)
{
    setTCAChannel(4);
    static const uint8_t image_KDU_logo_bits[] = {0x00, 0x18, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x3c, 0x00, 0x30, 0x7e, 0x0e, 0xc0, 0x7e, 0x03, 0x00, 0xbd, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x3e, 0x00, 0xf0, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xfe, 0xff, 0x3f, 0xc0, 0xff, 0x03, 0x3c, 0x7f, 0x3e, 0x3c, 0xff, 0x3c, 0x0c, 0x7e, 0x34, 0xe4, 0xbd, 0x27, 0xe0, 0xdb, 0x07, 0x20, 0xff, 0x04, 0x00, 0x1c, 0x00};
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont22_tr);
    u8g2_DrawStr(&u8g2, 16, 24, "NUCLEO-5");

    u8g2_DrawBox(&u8g2, 0, 0, 128, 33);

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawXBM(&u8g2, 52, 40, 24, 19, image_KDU_logo_bits);

    u8g2_SendBuffer(&u8g2);
}
void caliberate_OLED(void)
{
    setTCAChannel(4);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);
    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont22_tr);
    u8g2_DrawStr(&u8g2, 4, 39, "Caliberate");
    u8g2_DrawBox(&u8g2, 0, 17, 128, 30);
    u8g2_SendBuffer(&u8g2);
}

void TASK_1_OLED(void)
{
    setTCAChannel(4);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont22_tr);
    u8g2_DrawStr(&u8g2, 28, 39, "TASK-1");

    u8g2_DrawBox(&u8g2, 0, 17, 128, 30);

    u8g2_SendBuffer(&u8g2);
}
void TASK_2_OLED(void)
{
    setTCAChannel(4);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont22_tr);
    u8g2_DrawStr(&u8g2, 28, 39, "TASK-2");

    u8g2_DrawBox(&u8g2, 0, 17, 128, 30);

    u8g2_SendBuffer(&u8g2);
}
void TASK_3_OLED(void)
{
    setTCAChannel(4);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont22_tr);
    u8g2_DrawStr(&u8g2, 28, 39, "TASK-3");

    u8g2_DrawBox(&u8g2, 0, 17, 128, 30);

    u8g2_SendBuffer(&u8g2);
}
void TASK_4_OLED(void)
{
    setTCAChannel(4);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont22_tr);
    u8g2_DrawStr(&u8g2, 28, 39, "TASK-4");

    u8g2_DrawBox(&u8g2, 0, 17, 128, 30);

    u8g2_SendBuffer(&u8g2);
}
void TASK_5_OLED(void)
{
    setTCAChannel(4);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont22_tr);
    u8g2_DrawStr(&u8g2, 28, 39, "TASK-5");

    u8g2_DrawBox(&u8g2, 0, 17, 128, 30);

    u8g2_SendBuffer(&u8g2);
}
void TASK_6_OLED(void)
{
    setTCAChannel(4);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont22_tr);
    u8g2_DrawStr(&u8g2, 28, 39, "TASK-6");

    u8g2_DrawBox(&u8g2, 0, 17, 128, 30);

    u8g2_SendBuffer(&u8g2);
}
void display_TASK(int taskNumber)
{
    setTCAChannel(4);

    static const uint8_t image_KDU_logo_bits[] = {0x00, 0x18, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x3c, 0x00, 0x30, 0x7e, 0x0e, 0xc0, 0x7e, 0x03, 0x00, 0xbd, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x3e, 0x00, 0xf0, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xfe, 0xff, 0x3f, 0xc0, 0xff, 0x03, 0x3c, 0x7f, 0x3e, 0x3c, 0xff, 0x3c, 0x0c, 0x7e, 0x34, 0xe4, 0xbd, 0x27, 0xe0, 0xdb, 0x07, 0x20, 0xff, 0x04, 0x00, 0x1c, 0x00};
    char taskString[10]; // Buffer to hold the task string

    // Format the task string dynamically
    snprintf(taskString, sizeof(taskString), "TASK %d", taskNumber);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont22_tr);
    u8g2_DrawStr(&u8g2, 16, 24, "NUCLEO-5");

    u8g2_DrawBox(&u8g2, 1, 0, 128, 32);

    u8g2_SetFont(&u8g2, u8g2_font_profont17_tr);
    u8g2_DrawStr(&u8g2, 66, 54, taskString);

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawXBM(&u8g2, 8, 39, 24, 19, image_KDU_logo_bits);

    u8g2_DrawLine(&u8g2, 0, 0, 0, 0);

    u8g2_SendBuffer(&u8g2);
}
void display_barcode_decimal(int decimalValue)
{
    char decimalString[10];                                             // Buffer to hold the decimal string
    snprintf(decimalString, sizeof(decimalString), "%d", decimalValue); // Format the decimal value
    setTCAChannel(4);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);
    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_DrawBox(&u8g2, 0, 0, 128, 64);
    u8g2_SetFont(&u8g2, u8g2_font_inb57_mn);
    u8g2_DrawStr(&u8g2, 14, 60, decimalString); // Display the decimal value
    u8g2_SetFont(&u8g2, u8g2_font_profont29_tr);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawLine(&u8g2, 0, 0, 0, 0);
    u8g2_SendBuffer(&u8g2);
}
void alldone(void)
{
    setTCAChannel(4);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    u8g2_DrawBox(&u8g2, 0, 0, 128, 64);

    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont17_tr);
    u8g2_DrawStr(&u8g2, 10, 50, "ACCOMPLISHED");

    u8g2_SetFont(&u8g2, u8g2_font_profont29_tr);
    u8g2_DrawStr(&u8g2, 8, 31, "MISSION");

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawLine(&u8g2, 0, 0, 0, 0);

    u8g2_SendBuffer(&u8g2);
}
// Define menu states
typedef enum
{
    MENU_TASK_1,
    MENU_TASK_2,
    MENU_TASK_3,
    MENU_TASK_4,
    MENU_TASK_5,
    MENU_TASK_6
} OLED_MenuState;

OLED_MenuState currentMenuState = MENU_TASK_1; // Start with the first menu option

void UpdateOLEDMenu()
{
    if (currentMenuState == MENU_TASK_1)
    {
        TASK_1_OLED(); // Display "Task 1" on the OLED
        if (Button2 == 1)
        {
            start_task_1 = 1; // Start Task 1
            Button2 = 0;      // Reset the button state
        }
    }
    else if (currentMenuState == MENU_TASK_2)
    {
        TASK_2_OLED(); // Display "Task 2" on the OLED
        if (Button2 == 1)
        {
            start_task_2 = 1; // Start Task 2
            Button2 = 0;      // Reset the button state
        }
    }
    else if (currentMenuState == MENU_TASK_3)
    {
        TASK_3_OLED(); // Display "Task 3" on the OLED
        if (Button2 == 1)
        {
            start_task_3 = 1; // Start Task 3
            Button2 = 0;      // Reset the button state
        }
    }
    else if (currentMenuState == MENU_TASK_4)
    {
        TASK_4_OLED(); // Display "Task 4" on the OLED
        if (Button2 == 1)
        {
            start_task_4 = 1; // Start Task 4
            Button2 = 0;      // Reset the button state
        }
    }
    else if (currentMenuState == MENU_TASK_5)
    {
        TASK_5_OLED(); // Display "Task 5" on the OLED
        if (Button2 == 1)
        {
            start_task_5 = 1; // Start Task 5
            Button2 = 0;      // Reset the button state
        }
    }
    else if (currentMenuState == MENU_TASK_6)
    {
        TASK_6_OLED(); // Display "Task 6" on the OLED
        if (Button2 == 1)
        {
            start_task_6 = 1; // Start Task 6
            Button2 = 0;      // Reset the button state
        }
    }
}
void HandleButton1Press()
{
    // Toggle between Task 1 and Task 6
    if (currentMenuState == MENU_TASK_1)
    {
        currentMenuState = MENU_TASK_2;
    }
    else if (currentMenuState == MENU_TASK_2)
    {
        currentMenuState = MENU_TASK_3;
    }
    else if (currentMenuState == MENU_TASK_3)
    {
        currentMenuState = MENU_TASK_4;
    }
    else if (currentMenuState == MENU_TASK_4)
    {
        currentMenuState = MENU_TASK_5;
    }
    else if (currentMenuState == MENU_TASK_5)
    {
        currentMenuState = MENU_TASK_6;
    }
    UpdateOLEDMenu(); // Update the OLED display
}
void RED()
{
    setTCAChannel(4);

    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    u8g2_DrawBox(&u8g2, 0, 0, 128, 64);

    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont29_tr);
    u8g2_DrawStr(&u8g2, 41, 42, "RED");

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawLine(&u8g2, 0, 0, 0, 0);

    u8g2_SendBuffer(&u8g2);
}
void BLUE()
{
    setTCAChannel(4);

    u8g2_ClearBuffer(&u8g2);
    u8g2_SetBitmapMode(&u8g2, 1);
    u8g2_SetFontMode(&u8g2, 1);

    u8g2_DrawBox(&u8g2, 0, 0, 128, 64);

    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_profont29_tr);
    u8g2_DrawStr(&u8g2, 41, 42, "BLUE");

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawLine(&u8g2, 0, 0, 0, 0);

    u8g2_SendBuffer(&u8g2);
}