#include "task_6.h"
#include "state_machine.h"
#include "Timer_Delay.h"
#include "motor_control.h"
#include "encoders.h"
#include "controller.h"
#include "tcs34725.h"
#include "sensors.h"
#include "Timer_Delay.h"
#include "systick.h"
#include "controller.h"
#include "ultrasonic.h"
#include "pca9685.h"
#include "i2c_MUX.h"
#include "Actuators.h"
#include "oled_display.h"
#include <stdlib.h>

extern int DIST;
int align_to_plant = 0;
extern MainState currentMainState;
int plant_line_detected = 0; // Flag to indicate if the plant line is detected

Task6SubState currentTask6SubState = STATE_Display;

void ultrasonic_startTSK6(int sonic)
// 0 left Ultraonic
// 1 right Ultraonic
// 2 front Ultraonic
{
    resetEncoders();
    for (int i = 0; i < 10; i++)
    {
        DIST = Ultrasonic_GetDistance(sonic);
        delay_ms(20);
    }
}
void task6StateHandler()
{
    switch (currentTask6SubState)
    {
    case STATE_Display:
        display_TASK(6);
        currentTask6SubState = STATE_entering;

        break;
    case STATE_entering:
        enc_drive();
        if (IR[7] == 1 && IR[8] == 1)
        {
            enc_drive_decel(730, 80, 65);
            enc_drive_T(1660, 80, 75);
            delay_ms(20);
            turn_Left_90_LR();
            delay_ms(20);
            enc_drive_T(1464, 80, 75);
            turn_Right_90_LR_Controlled(850, 80, 70);

            delay_ms(20);
            currentTask6SubState = STATE_Plant_line_detection;
        }
        break;
    case STATE_Plant_line_detection:
        enc_drive();
        if (IR[8] == 1)
        {
            enc_drive_decel(730, 80, 65);
            plant_line_detected++;

            if (plant_line_detected == 2)
            {

                turn_Right_90_LR_Controlled(830, 80, 70);
                delay_ms(20);
                currentTask6SubState = STATE_2ndplant_line_detected;
            }
        }
        else
        {
            enc_drive();
        }
        break;
    case STATE_2ndplant_line_detected:
        enc_drive();
        if (IR[3] == 1)
        {
            enc_drive_decel(62, 80, 65);
            turn_90_degreesR1_reverse(800);
            enc_drive_T(1400, 80, 65);
            turn_90_degreesR1_reverse(850);
            enc_driveR_T(1200, 80, 65);
            currentTask6SubState = STATE_TeST;
        }
        break;
    case STATE_PlantTLine_detection:
        enc_drive();
        if (IR[0] == 1 || IR[1] == 1 || IR[2] == 1 || IR[3] == 1 || IR[4] == 1 || IR[5] == 1 || IR[6] == 1 || IR[7] == 1 || IR[8] == 1)
        {
            enc_drive_decel(124, 80, 65);
            m_stopLR();
            delay_ms(20);
            resetEncoders();
            currentTask6SubState = STATE_Plant_T_detection;
        }
        else
        {
            enc_drive();
        }
        break;
    case STATE_Plant_T_detection:
        line_follow();
        if (IR[0] == 1 && IR[1] == 1 && IR[2] == 1 && IR[3] == 1 && IR[4] == 1)
        {
            enc_drive_decel(62, 80, 65);
            m_stopLR();
            delay_ms(20);
            resetEncoders();
            m_stopLR();

            turn_90_degreesR1_reverse(800);
            enc_drive_T(1400, 80, 65);
            turn_90_degreesR1_reverse(850);
            enc_driveR_T(900, 80, 65);
            delay_ms(20);
            water_pump_mid_to_down();
            delay_ms(1000);

            currentTask6SubState = STATE_Plant_Check;
        }
        else
        {
            line_follow();
        }
        break;
    case STATE_Plant_Check:
        if (IR[9] == 0)
        {
            pump_1_on();
            delay_ms(6000);
            pump_1_off();
            delay_ms(800);
            water_pump_down_to_mid();
            delay_ms(1000);
            currentTask6SubState = STATE_Done;
        }
        else
        {
            water_pump_down_to_mid();
            delay_ms(1000);
            enc_driveR_T(2420, 80, 65);
            delay_ms(1000);
            water_pump_mid_to_down();
            delay_ms(1000);

            if (IR[9] == 0)
            {
                pump_1_on();
                delay_ms(6000);
                pump_1_off();
                water_pump_down_to_mid();
                delay_ms(1000);
                currentTask6SubState = STATE_Done;
            }
            else
            {
                water_pump_down_to_mid();
                delay_ms(1000);
                enc_drive_T(4500, 80, 65);
                delay_ms(1000);
                water_pump_mid_to_down();
                delay_ms(1000);
                if (IR[9] == 0)
                {
                    pump_1_on();
                    delay_ms(6000);
                    pump_1_off();
                    water_pump_down_to_mid();
                    delay_ms(1000);
                    currentTask6SubState = STATE_Done;
                }
                else
                {
                    water_pump_down_to_mid();
                    delay_ms(1000);

                    currentTask6SubState = STATE_Done;
                }
            }
        }

        break;
    case STATE_Done:
        turn_Right_90_LR();
        delay_ms(20);
        enc_drive_T(1800, 80, 65);
        alldone();
        delay_ms(20000);

        break;
    case STATE_TeST:
        setTCAChannel(2);
        delay_ms(1000);
        PCA9685_SetServoAngle(14, 109);
        pump_2_on();
        delay_ms(4000);
        pump_2_off();
        delay_ms(1000);
        currentTask6SubState = STATE_Arm_Down;
        break;
    case STATE_Arm_Down:
        setTCAChannel(2);
        PCA9685_Init(50);
        PCA9685_SetServoAngle(14, 0); // 170
        setTCAChannel(2);
        PCA9685_Init(50);
        PCA9685_SetServoAngle(14, 0); // 170
        setTCAChannel(2);
        PCA9685_Init(50);
        PCA9685_SetServoAngle(14, 0); // 170
        setTCAChannel(2);
        PCA9685_Init(50);
        PCA9685_SetServoAngle(14, 0); // 170
        setTCAChannel(2);
        PCA9685_Init(50);
        PCA9685_SetServoAngle(14, 0); // 170
        setTCAChannel(2);
        PCA9685_Init(50);
        PCA9685_SetServoAngle(14, 0); // 170
        setTCAChannel(2);
        PCA9685_Init(50);
        PCA9685_SetServoAngle(14, 0); // 170
        setTCAChannel(2);
        PCA9685_Init(50);
        PCA9685_SetServoAngle(14, 0); // 170
        setTCAChannel(2);
        PCA9685_Init(50);
        PCA9685_SetServoAngle(14, 0); // 170
        water_pump_mid();

        delay_ms(1000);
        enc_drive_T(750, 80, 65);
        turn_Right_90_LR();
        currentTask6SubState = STATE_PlantTLine_detection;
        break;
    }
}
