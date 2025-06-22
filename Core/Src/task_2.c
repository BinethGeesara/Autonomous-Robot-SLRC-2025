#include "task_2.h"
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

extern int enc_dist;
extern MainState currentMainState;

int T2_A_Detected = 0;
int T2_B_Detected = 0;
int T2_Straight_object_detected = 0;
int ultr_int = 0;
int Distance4;
int Distance3;
extern volatile uint8_t irInterruptFlag;
extern volatile uint8_t rightIRTriggered;
extern volatile uint8_t leftIRTriggered;
int left_adjust = 0;
int right_adjust = 0;
extern int frontwall_detected;
extern int frontwall_Distance;
int path_A = 0;
int path_B = 0;
extern int min_sidewall_Distance;
extern int sidewall_Distance;
int leftside_of_ramp = 0;
int rightside_of_ramp = 0;

Task2SubState currentTask2SubState = STATE_TASK2_A;

void task2StateHandler()
{

    switch (currentTask2SubState)
    {
    case STATE_TASK2_A:
        display_TASK(2);
        if (ultr_int == 0)
        {
            resetEncoders();
            for (int i = 0; i < 10; i++)
            {
                sidewall_Distance = Ultrasonic_GetDistance(2);
                delay_ms(50);
            }
            ultr_int = 1;
        }
        sidewall_Distance = Ultrasonic_GetDistance(2);
        enc_drive_with_ultrasonic_Target(1860);
        enc_drive_decel(156, 70, 60);
        m_stopLR();
        delay_ms(300);
        Distance3 = min_sidewall_Distance;
        currentTask2SubState = STATE_TASK2_B;
        break;
    case STATE_TASK2_B:
        if (ultr_int == 0)
        {
            resetEncoders();
            for (int i = 0; i < 10; i++)
            {
                sidewall_Distance = Ultrasonic_GetDistance(2);
                delay_ms(50);
            }
            ultr_int = 1;
        }
        sidewall_Distance = Ultrasonic_GetDistance(2);
        enc_drive_with_ultrasonic_Target(516);
        enc_drive_decel(156, 70, 60);
        m_stopLR();
        delay_ms(300);

        Distance4 = min_sidewall_Distance;
        if (Distance3 < Distance4)
        {
            turn_90_degreesR1();
            path_B = 1;
        }
        else  
        {
            enc_driveR_T(1889, 70, 60);
            m_stopLR();
            delay_ms(30);
            turn_90_degreesR1();
            path_A = 1;
        }
        currentTask2SubState = STATE_TASK2_A1;
        break;
    case STATE_TASK2_A1:
        if (ultr_int == 0)
        {
            resetEncoders();
            for (int i = 0; i < 10; i++)
            {
                frontwall_Distance = Ultrasonic_GetDistance(1);
                delay_ms(30);
            }
            ultr_int = 1;
        }
        enc_drive_with_ultrasonic_check();
        if (frontwall_detected == 1)
        {
            enc_drive_decel(156, 70, 60);
            m_stopLR();
            delay_ms(30);
            if (path_B == 1)
            {
                path_B = 0;
                turn_Right_45_TASK2_LR();
                m_stopLR();
                delay_ms(20);
                enc_drive_T(1986, 80, 60);
                m_stopLR();
                delay_ms(20);
                turn_Left_45_TASK2_LR();
                m_stopLR();
                delay_ms(50);
                rightside_of_ramp = 1;
               /*enc_drive_T(1400, 80, 60);
                m_stopLR();
                delay_ms(20);
                turn_Left_45_TASK2_LR();
                m_stopLR();
                delay_ms(20);
                enc_drive_T(450, 80, 60);
                m_stopLR();
                delay_ms(20);
                turn_Right_45_TASK2_LR();
                m_stopLR();
                delay_ms(20);*/ 
            }
            else
            {
                path_A = 0;
                turn_Left_45_TASK2_LR();
                m_stopLR();
                delay_ms(20);
                enc_drive_T(1614, 80, 60);
                m_stopLR();
                delay_ms(20);
                turn_Right_45_TASK2_R1();
                m_stopLR();
                delay_ms(50);
                leftside_of_ramp = 1;
              /*  enc_drive_T(1400, 80, 60);
                m_stopLR();
                delay_ms(20);
                turn_Right_45_TASK2_LR();
                m_stopLR();
                delay_ms(20);
                enc_drive_T(450, 80, 60);
                m_stopLR();
                delay_ms(20);
                turn_Left_45_TASK2_LR();
                m_stopLR();
                delay_ms(20);*/ 
            }
        }
        resetEncoders();
        currentMainState = TASK_3;
        break;
    }
    // End of switch (currentTask2SubState)
}
