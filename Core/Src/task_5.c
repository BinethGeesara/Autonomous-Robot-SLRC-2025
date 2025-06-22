#include "task_5.h"
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

extern MainState currentMainState;

Task5SubState currentTask5SubState = STATE_display;
int task5 = 0; // Task 5 not completed
int DIST5 = 0;
int pad2 = 0; // Flag to indicate if the second pad is detected
void ultrasonic_startTS5(int n)
// 0 left Ultraonic
// 1 right Ultraonic
// 2 front Ultraonic
{
    resetEncoders();
    for (int i = 0; i < 10; i++)
    {
        DIST5 = Ultrasonic_GetDistance(n);
        delay_ms(20);
    }
}
void boxPlaceTS5()
{
    BOX_Arm_Down();
    BOX_Gripper_Open();
    BOX_Arm_Up();
    BOX_Gripper_Close();
}

void task5StateHandler()
{
    switch (currentTask5SubState)
    {
    case STATE_display:
        display_TASK(5);
        currentTask5SubState = STATE_Line_Detect;
        break;
    case STATE_Line_Detect:
        line_follow();
        if (IR[7] == 1)
        {
            enc_drive_decel(750, 80, 65);
            m_stopLR();
            delay_ms(80);
            resetEncoders();
            turn_Right_90_LR();
            currentTask5SubState = STATE_right;
        }
        break;
    case STATE_right:
        line_follow();
        if (IR[0] == 1 && IR[1] == 1 && IR[2] == 1 && IR[3] == 1 && IR[4] == 1 && IR[5] == 1 && IR[6] == 1)

        {
            enc_drive_decel(62, 85, 60);
            setTCAChannel(0);
            delay_ms(10);
            for (int i = 0; i < 10; i++)
            {
                Bottom_C_PAD_Sensor();
                delay_ms(20);
            }
            if (bottomColor == COLOR_RED)
            {
                enc_driveR_T(250, 80, 60);
                boxPlaceTS5();
                turn_Right_180_LR();
                line_follow_to_target(1000, 80, 60);
                task5 = 1; // Task 5 completed
            }
            else
            {
                turn_Right_180_LR();
                task5 = 0;
            }
            if (task5 == 1)
            {
                currentMainState = TASK_6;
            }
            else
            {
                currentTask5SubState = STATE_left;
            }
        }
        break;
    case STATE_left:
        line_follow();
        if (IR[7] == 1 && IR[8] == 1)
        {
            enc_drive_decel(750, 80, 65);
            m_stopLR();
            delay_ms(80);
            resetEncoders();
            turn_Right_90_LR();
            currentTask5SubState = STATE_pad2;
        }
        break;
    case STATE_pad2:
        line_follow();
         if (IR[0] == 1 && IR[1] == 1 && IR[2] == 1 && IR[3] == 1 && IR[4] == 1 && IR[5] == 1 && IR[6] == 1)

        {
            enc_drive_decel(62, 85, 60);
            enc_driveR_T(250, 80, 60);
            boxPlaceTS5();
            turn_Right_180_LR();
            currentTask5SubState = STATE_3;
        }
        break;
    case STATE_3:
        line_follow();
        if (IR[8] == 1)
        {
            enc_drive_decel(750, 80, 65);
            m_stopLR();
            delay_ms(80);
            resetEncoders();
            turn_Right_90_LR();
            line_follow_to_target(1000, 80, 60);
            currentMainState = TASK_6;
        }
        break;
    }
}