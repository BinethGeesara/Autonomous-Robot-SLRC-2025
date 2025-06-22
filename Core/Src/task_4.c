#include "task_4.h"
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
#include <stdio.h> // For debugging/logging purposes
// whl axis to box scan lenght 372
// ultrasonic to box scan lenght  1209
// ultrasonic to ultrasonic cell length to
// ultrasonic to whls axis 868
// ultrasonic box scan lenght to ultrasonic cell 620

int grid[3][3] = {0};      // Initialize all cells to 0
int visited[3][3] = {0};   // Initialize all cells to 0
int red_array[3][3] = {0}; // Initialize all cells to 0
int ultr_int3 = 0;
int DIST = 0;
int red_picked = 0;
int imon10 = 0;
int goto_1_ROW = 0;
int goto_2_ROW = 0;

extern MainState currentMainState;

Task4SubState currentTask4SubState = STATE_linefollow;

/// @brief ////////////////////
/// @param sonic /
void ultrasonic_start(int sonic)
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
void box_detect(int sonic, int x, int y)
{
    ultrasonic_start(sonic);
    if (DIST < 180)
    {
        grid[x][y] = 1; // Mark the box as detected
        DIST = 0;       // Reset the distance for the next check
    }
    else
    {
        grid[x][y] = 0; // Mark the box as not detected
        DIST = 0;       // Reset the distance for the next check
    }
}
void box_detect_Custom(int sonic, int x, int y, int length)
{
    ultrasonic_start(sonic);
    if (DIST < length)
    {
        grid[x][y] = 1; // Mark the box as detected
        DIST = 0;       // Reset the distance for the next check
    }
    else
    {
        grid[x][y] = 0; // Mark the box as not detected
        DIST = 0;       // Reset the distance for the next check
    }
}
void colorScan()
{
    setTCAChannel(1);
    delay_ms(10);
    for (int i = 0; i < 10; i++)
    {
        Top_C_BOX_Sensor();
        delay_ms(20);
    }
}
void boxPick()
{
    BOX_Gripper_Open();
    BOX_Arm_Down();
    BOX_Gripper_Close();
    BOX_Arm_Up();
}
void boxPlace()
{
    BOX_Arm_Down();
    BOX_Gripper_Open();
    BOX_Arm_Up();
    BOX_Gripper_Close();
}
//////////////////////////////
void L90()
{
    T4_turn_Left_90_LR();
    m_stopLR();
    delay_ms(20);
}
void R90()
{
    T4_turn_Right_90_LR();
    m_stopLR();
    delay_ms(20);
}
void L180()
{
    T4_turn_Left_180_LR();
    m_stopLR();
    delay_ms(20);
}
//////////////////////////////
void to_firsROW()
{ // ultrasonic to whls axis 868
    resetEncoders();
    enc_drive_T(900, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void ultr_to_whl()
{ // ultrasonic to whls axis 868
    resetEncoders();
    enc_drive_T(868, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void box_scan_to_whl()
{
    resetEncoders();
    enc_drive_T(1300, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void whl_to_box_scan()
{ // whl axis to box scan lenght 372
    resetEncoders();
    enc_drive_T(360, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void whl_to_box_scan_Reverse()
{ // whl axis to box scan lenght 372
    resetEncoders();
    enc_driveR_T(360, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void ultr_box_scan_TO_ultr_cell()
{ // ultrasonic box scan lenght to ultrasonic cell 620
    resetEncoders();
    enc_drive_T(510, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void whl_to_ultr_cell()
{ // whl axis to eltr on cell
    resetEncoders();
    enc_drive_T(858, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void whl_to_ultr_cell_Reverse()
{ // whl axis to eltr on cell
    resetEncoders();
    enc_driveR_T(930, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void whl_to_whl()
{
    resetEncoders();
    enc_drive_T(1755, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void whl_to_whl_reverse()
{
    resetEncoders();
    enc_driveR_T(1738, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void ultr_cell_to_whl_on_cell()
{
    resetEncoders();
    enc_drive_T(878, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void ontoCell_2_1()
{
    resetEncoders();
    enc_driveR_T(1382, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
///////////////////////////////
void to_cell_0_1()
{
    resetEncoders();
    ultr_to_whl();
    R90();
    to_firsROW();
    if (grid[0][0] == 1)
    {
        R90();
        whl_to_box_scan();
        colorScan();
        if (boxColor == COLOR_BLUE)
        {
            BLUE();
            whl_to_box_scan_Reverse();
            L90();
        }
        else
        {
            RED();
            boxPick();
            red_picked = 1;
            whl_to_box_scan_Reverse();
            R90();
            whl_to_ultr_cell_Reverse();
        }
    }
    if (red_picked == 0)
    {
        L90();
        box_detect(1, 0, 2);
        if (grid[0][2] == 1)
        {
            whl_to_box_scan();
            colorScan();
            if (boxColor == COLOR_BLUE)
            {
                whl_to_box_scan_Reverse();
                R90();
                goto_1_ROW = 1;
            }
            else
            {
                RED();
                boxPick();
                red_picked = 1;
                whl_to_box_scan_Reverse();
                L90();
                whl_to_ultr_cell_Reverse();
            }
        }
        else
        {
            R90();
            goto_1_ROW = 1;
        }
    }
    if (red_picked == 1)
    {
        currentTask4SubState = FinishToline;
    }
    else if (goto_1_ROW == 1)
    {
        resetEncoders();
        currentTask4SubState = STATE_1_ROW;
    }
}
void to_cell_0_0()
{
    resetEncoders();
    enc_driveR_T(825, 80, 60);
    m_stopLR();
    delay_ms(20);
    resetEncoders();
    R90();
    to_firsROW();
    L90();
    whl_to_box_scan();
    colorScan();
    if (boxColor == COLOR_BLUE)
    {
        BLUE();
    }
    else
    {
        RED();
    }
    boxPick();
    grid[0][1] = 0;
    box_scan_to_whl();
    box_detect(1, 0, 2);

    if (boxColor == COLOR_BLUE)
    {
        BLUE();
        L180();
        whl_to_box_scan();
        boxPlace();
        grid[0][0] = 1;
        visited[0][0] = 1;
        whl_to_box_scan_Reverse();
        L90();
        if (grid[0][2] == 1 && red_picked == 0 && visited[0][2] == 0)
        {

            L90();
            whl_to_box_scan();
            colorScan();
            if (boxColor == COLOR_BLUE)
            {
                BLUE();
                whl_to_box_scan_Reverse();
                R90();
                goto_1_ROW = 1;
            }
            else
            {
                RED();
                red_picked = 1;
                boxPick();
                whl_to_box_scan_Reverse();
                L90();
            }
        }
        else
        {
            goto_1_ROW = 1;
        }
        ///////////////// go to 0,2 ans scan and and if blue come to 0,1 , if red pick and exit
    }
    else
    {
        RED();
        red_picked = 1;
        L90();
        whl_to_ultr_cell_Reverse();
    }
    if (red_picked == 1)
    {
        currentTask4SubState = FinishToline;
    }
    else if (goto_1_ROW == 1)
    {
        resetEncoders();
        currentTask4SubState = STATE_1_ROW;
    }
}
void to_cell_1_1_noBOX()
{
    whl_to_whl();
    R90();
    box_detect(1, 1, 0);
    if (grid[1][0] == 1)
    {
        whl_to_box_scan();
        colorScan();
        if (boxColor == COLOR_BLUE)
        {
            BLUE();
            whl_to_box_scan_Reverse();
            L90();
        }
        else
        {
            RED();
            boxPick();
            red_picked = 1;
            whl_to_box_scan_Reverse();
            R90();
        }
    }
    else
    {
        L90();
    }
    if (red_picked == 0)
    {
        L90();
        box_detect(1, 1, 2);
        if (grid[1][2] == 1)
        {
            whl_to_box_scan();
            colorScan();
            if (boxColor == COLOR_BLUE)
            {
                whl_to_box_scan_Reverse();
                R90();
                goto_2_ROW = 1;
            }
            else
            {
                RED();
                boxPick();
                red_picked = 1;
                whl_to_box_scan_Reverse();
                L90();
            }
        }
        else
        {
            R90();
            goto_2_ROW = 1;
        }
    }
    if (red_picked == 1)
    {
        currentTask4SubState = FinishToline;
    }
    else if (goto_2_ROW == 1)
    {
        resetEncoders();
        currentTask4SubState = STATE_2_ROW;
    }
}
void to_cell_1_1_BOX()
{
    whl_to_box_scan();
    colorScan();
    if (boxColor == COLOR_BLUE)
    {
        BLUE();
        boxPick();
        grid[1][1] = 0;
        box_scan_to_whl();
        R90();
        box_detect(1, 1, 0);

        if (grid[1][0] == 1)
        {
            whl_to_box_scan();
            colorScan();
            if (boxColor == COLOR_RED)
            {
                RED();
                red_array[1][0] = 1;
            }
            whl_to_box_scan_Reverse();
            L180();
            box_detect(1, 1, 2);
            if (grid[1][2] == 1)
            {
                whl_to_box_scan();
                colorScan();
                if (boxColor == COLOR_RED)
                {
                    RED();
                    red_array[1][2] = 1;
                }
                whl_to_box_scan_Reverse();
                if (grid[0][0] == 1 && grid[0][2] == 1)
                {
                    R90();
                    whl_to_box_scan();
                    boxPlace();
                    grid[2][1] = 1;
                    whl_to_box_scan_Reverse();
                    if (red_array[1][0] == 1)
                    {
                        R90();
                        whl_to_box_scan();
                        boxPick();
                        red_picked = 1;
                        whl_to_box_scan_Reverse();
                        R90();
                    }
                    else if (red_array[1][2] == 1)
                    {
                        L90();
                        whl_to_box_scan();
                        boxPick();
                        red_picked = 1;
                        whl_to_box_scan_Reverse();
                        L90();
                    }
                }
                else if (grid[0][0] == 0 || grid[0][2] == 0)
                {
                    L90();
                    whl_to_whl();
                    if (grid[0][0] == 0)
                    {
                        L90();
                        whl_to_box_scan();
                        boxPlace();
                        grid[0][0] = 1;
                        whl_to_box_scan_Reverse();
                        L90();
                        whl_to_whl();
                    }
                    else if (grid[0][2] == 0)
                    {
                        R90();
                        whl_to_box_scan();
                        boxPlace();
                        grid[0][2] = 1;
                        whl_to_box_scan_Reverse();
                        R90();
                        whl_to_whl();
                    }
                    if (red_array[1][0] == 1)
                    {
                        R90();
                        whl_to_box_scan();
                        boxPick();
                        red_picked = 1;
                        whl_to_box_scan_Reverse();
                        R90();
                    }
                    else if (red_array[1][2] == 1)
                    {
                        L90();
                        whl_to_box_scan();
                        boxPick();
                        red_picked = 1;
                        whl_to_box_scan_Reverse();
                        L90();
                    }
                    else
                    {
                        goto_2_ROW = 1;
                    }
                }
            }
            else
            {
                whl_to_box_scan();
                boxPlace();
                grid[1][2] = 1;
                whl_to_box_scan_Reverse();
                R90();
                if (red_array[1][0] == 1)
                {
                    R90();
                    whl_to_box_scan();
                    boxPick();
                    red_picked = 1;
                    whl_to_box_scan_Reverse();
                    R90();
                }
                else
                {
                    goto_2_ROW = 1;
                }
            }
        }
        else
        {
            whl_to_box_scan();
            boxPlace();
            grid[1][0] = 1;
            whl_to_box_scan_Reverse();
            L180();
            box_detect(1, 1, 2);
            if (grid[1][2] == 1)
            {
                whl_to_box_scan();
                colorScan();
                if (boxColor == COLOR_BLUE)
                {
                    BLUE();
                    whl_to_box_scan_Reverse();
                    R90();
                    goto_2_ROW = 1;
                }
                else
                {
                    RED();
                    boxPick();
                    red_picked = 1;
                    red_array[1][2] = 1;
                    whl_to_box_scan_Reverse();
                    L90();
                }
            }
            else
            {
                R90();
                goto_2_ROW = 1;
            }
        }
    }
    else
    {
        RED();
        red_picked = 1;
        boxPick();
        L180();
    }
    if (red_picked == 1)
    {
        currentTask4SubState = FinishToline;
    }
    else if (goto_2_ROW == 1)
    {
        resetEncoders();
        currentTask4SubState = STATE_2_ROW;
    }
}
void to_cell_2_1_noBOX()
{
    whl_to_box_scan();
    L180();
    ontoCell_2_1();
    L90();
    box_detect(1, 2, 0);
    if (grid[2][0] == 1)
    {
        whl_to_box_scan();
        colorScan();
        if (boxColor == COLOR_BLUE)
        {
            /*if bot on 2nd row and no
            box on 2,1 and blue box on 2,0 red box
            is defenitely on 2,2*/
            BLUE();
            whl_to_box_scan_Reverse();
            R90();
            R90();
            whl_to_box_scan();
            boxPick();
            red_picked = 1;
            whl_to_box_scan_Reverse();
            L90();
        }
        else
        {
            RED();
            boxPick();
            red_picked = 1;
            whl_to_box_scan_Reverse();
            R90();
        }
    }
    else
    {
        /*if bot on 2nd row and no
        box on 2,1 and no box on 2,0 red box
        is defenitely on2,2*/
        R90();
        R90();
        whl_to_box_scan();
        colorScan();
        if (boxColor == COLOR_BLUE)
        {
            BLUE();
        }
        else
        {
            RED();
        }
        boxPick();
        red_picked = 1;
        whl_to_box_scan_Reverse();
        L90();
    }
    if (red_picked == 1)
    {
        currentTask4SubState = FinishToline;
    }
}
void to_cell_2_1_BOX()
{
    whl_to_box_scan();
    colorScan();
    boxPick();
    grid[2][1] = 0;
    L180();
    if (boxColor == COLOR_RED)
    {
        RED();
        red_array[2][1] = 1;
        red_picked = 1;
    }
    else
    { // Blue box on hand
        ontoCell_2_1();
        L90();
        box_detect(1, 2, 0);
        if (grid[2][0] == 1)
        {
            whl_to_box_scan();
            colorScan();
            if (boxColor == COLOR_BLUE)
            {
                /*if blue box on 2,0 then
                RED box definitely on 2,2*/
                BLUE();
                whl_to_box_scan_Reverse();
                red_array[2][2] = 1;
                grid[2][2] = 1;
                R90();
                whl_to_whl();
                if (grid[1][0] == 0)
                {
                    L90();
                    whl_to_box_scan();
                    boxPlace();
                    grid[1][0] = 1;
                    whl_to_box_scan_Reverse();
                    L90();
                    whl_to_box_scan();
                    L180();
                    ontoCell_2_1();
                    R90();
                    whl_to_box_scan();
                    boxPick();
                    red_picked = 1;
                    whl_to_box_scan_Reverse();
                    L90();
                }
                else if (grid[1][2] == 0)
                {
                    R90();
                    whl_to_box_scan();
                    boxPlace();
                    grid[1][2] = 1;
                    whl_to_box_scan_Reverse();
                    R90();
                    whl_to_box_scan();
                    L180();
                    ontoCell_2_1();
                    R90();
                    whl_to_box_scan();
                    boxPick();
                    red_picked = 1;
                    whl_to_box_scan_Reverse();
                    L90();
                }
                else if (grid[1][2] == 1 && grid[1][0] == 1)
                {
                    whl_to_whl();
                    if (grid[0][0] == 0)
                    {
                        L90();
                        whl_to_box_scan();
                        boxPlace();
                        grid[0][0] = 1;
                        whl_to_box_scan_Reverse();
                        L90();
                        whl_to_whl();
                        whl_to_box_scan();
                        L180();
                        ontoCell_2_1();
                        R90();
                        whl_to_box_scan();
                        boxPick();
                        red_picked = 1;
                        whl_to_box_scan_Reverse();
                        L90();
                    }
                    else
                    {
                        R90();
                        whl_to_box_scan();
                        boxPlace();
                        grid[0][2] = 1;
                        whl_to_box_scan_Reverse();
                        R90();
                        whl_to_whl();
                        whl_to_box_scan();
                        L180();
                        ontoCell_2_1();
                        R90();
                        whl_to_box_scan();
                        boxPick();
                        red_picked = 1;
                        whl_to_box_scan_Reverse();
                        L90();
                    }
                }
            }
            else
            {
                RED();
                red_array[2][0] = 1;
                whl_to_box_scan_Reverse();
                grid[2][0] = 1;
                R90();
                R90();
                box_detect(1, 2, 2);
                if (grid[2][2] == 0)
                {
                    whl_to_box_scan();
                    boxPlace();
                    grid[2][2] = 1;
                    whl_to_box_scan_Reverse();
                    L180();
                    whl_to_box_scan();
                    boxPick();
                    red_picked = 1;
                    whl_to_box_scan_Reverse();
                    R90();
                }
                else
                {
                    /*Now the grid[2][2] == 1 cant place the
                    picked box on 3rd ROW*/
                    L90();
                    whl_to_whl();
                    if (grid[1][0] == 0)
                    {
                        L90();
                        whl_to_box_scan();
                        boxPlace();
                        grid[1][0] = 1;
                        whl_to_box_scan_Reverse();
                        L90();
                        whl_to_box_scan();
                        L180();
                        ontoCell_2_1();
                        L90();
                        whl_to_box_scan();
                        boxPick();
                        red_picked = 1;
                        whl_to_box_scan_Reverse();
                        R90();
                    }
                    else if (grid[1][2] == 0)
                    {
                        R90();
                        whl_to_box_scan();
                        boxPlace();
                        grid[1][2] = 1;
                        whl_to_box_scan_Reverse();
                        R90();
                        whl_to_box_scan();
                        L180();
                        ontoCell_2_1();
                        L90();
                        whl_to_box_scan();
                        boxPick();
                        red_picked = 1;
                        whl_to_box_scan_Reverse();
                        R90();
                    }
                    else if (grid[1][2] == 1 && grid[1][0] == 1)
                    {
                        whl_to_whl();
                        if (grid[0][0] == 0)
                        {
                            L90();
                            whl_to_box_scan();
                            boxPlace();
                            grid[0][0] = 1;
                            whl_to_box_scan_Reverse();
                            L90();
                            whl_to_whl();
                            whl_to_box_scan();
                            L180();
                            ontoCell_2_1();
                            L90();
                            whl_to_box_scan();
                            boxPick();
                            red_picked = 1;
                            whl_to_box_scan_Reverse();
                            R90();
                        }
                        else
                        {
                            R90();
                            whl_to_box_scan();
                            boxPlace();
                            grid[0][2] = 1;
                            whl_to_box_scan_Reverse();
                            R90();
                            whl_to_whl();
                            whl_to_box_scan();
                            L180();
                            ontoCell_2_1();
                            L90();
                            whl_to_box_scan();
                            boxPick();
                            red_picked = 1;
                            whl_to_box_scan_Reverse();
                            R90();
                        }
                    }
                }
            }
        }
        else
        {
            whl_to_box_scan();
            boxPlace();
            grid[2][0] = 1;
            whl_to_box_scan_Reverse();
            R90();
            R90();
            whl_to_box_scan();
            boxPick();
            red_picked = 1;
            whl_to_box_scan_Reverse();
            L90();
        }
    }
    if (red_picked == 1)
    {
        currentTask4SubState = FinishToline;
    }
}
///////////
void task4StateHandler()
{
    switch (currentTask4SubState)
    {
    case STATE_linefollow:
        display_TASK(4);
        while (IR[7] == 0)
        {
            line_follow_NoJunctions();
        }
        resetEncoders();
        enc_drive_decel(750, 80, 65);
        m_stopLR();
        delay_ms(20);
        currentTask4SubState = STATE_0_ROW;
        break;
    case STATE_0_ROW:
        box_detect_Custom(2, 0, 0, 200);
        line_follow_to_target(1736, 80, 60);
        box_detect_Custom(2, 0, 1, 200);

        if (grid[0][1] == 0)
        {
            to_cell_0_1();
        }
        else
        {
            to_cell_0_0();
        }
        break;
    case STATE_1_ROW:
        box_detect(1, 1, 1);
        if (grid[1][1] == 1)
        {
            to_cell_1_1_BOX();
        }
        else
        {
            to_cell_1_1_noBOX();
        }
        break;
    case STATE_2_ROW:
        box_detect(1, 2, 1);
        if (grid[2][1] == 1)
        {
            imon10 = 1;
            to_cell_2_1_BOX();
        }
        else
        {
            imon10 = 11;
            to_cell_2_1_noBOX();
        }
        break;
    case FinishTASK4:
        enc_drive2(130);
        if (IR[7] == 1 && IR[8] == 1)
        {
            enc_drive_decel(2350, 130, 55);
            m_stopLR();
            delay_ms(50); //*****tsk done*******/
            currentMainState = TASK_5;
        }
        break;
    case FinishToline:
        delay_ms(20);
        resetEncoders();
        currentTask4SubState = FinishTASK4;
        break;
    case STATE_TASK4:
        boxPick();
        delay_ms(6000);
        // to_cell_2_1();
        break;
        // Implementation of task 4 state handling logic goes here
    }
}
