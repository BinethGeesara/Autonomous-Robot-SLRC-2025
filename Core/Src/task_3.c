#include "task_3.h"
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

#define BARCODE_END_THRESHOLD 350 // Encoder count threshold to detect the end of the barcode
#define MAX_BARCODE_BITS 8        // Maximum number of bits in the barcode for live debugging
int work = 0;
extern int wall;
extern MainState currentMainState;

int currentStripeValue = -1;
int barcodeDetected = 0;           // Holds the current stripe value being processed
int barcodeComplete = 0;           // Flag to indicate when the barcode is complete
int barcodeData[MAX_BARCODE_BITS]; // Fixed-size array to store barcode bits for live debugging
int barcodeSize = 0;               // Number of bits in the barcode
int decimalValue = 0;
int stripeValue;
int stripeWidth = 0; // Variable to store the width of the stripe
int ultr_int2 = 0;
int distance = 0;
int TSK3_first_PAD = 0; // 0 = red ,1 Blue
int TSK3_BLUE_PAD = 0;

int Y_balls_to_RED = 0;  // Number of yellow balls to drop on the red pad
int Y_balls_to_BLUE = 0; // Number of red balls to drop on the blue pad
int other_ball = 0;      // 0 = yellow , 1 = white
extern int frontwall_detected;
extern int frontwall_Distance;
extern int obstacle_detected;
extern int leftside_of_ramp;
extern int rightside_of_ramp;
void detectBarCode()
{
    static int isWhiteStripe = 0;        // Tracks if the robot is on a white stripe
    static int stripeStartLeftCount = 0; // Left encoder count at the start of the stripe
    static int stripeStartRightCount = 0;
    static int lastDetectionLeftCount = 0; // Tracks the last encoder count when IR[3] was detected

    // Cache IR sensor value (updated via DMA)
    int irValue = IR[3];
    int currentLeftCount = getLeftEncoderCounts(); // Get the current left encoder count

    if (irValue == 1) // IR sensor detects a white stripe
    {
        barcodeDetected = 1;                       // Set the barcode detected flag
        lastDetectionLeftCount = currentLeftCount; // Update the last detection count

        if (!isWhiteStripe) // Only reset and start when entering a white stripe
        {
            isWhiteStripe = 1;

            // Cache encoder counts at the start of the stripe
            stripeStartLeftCount = currentLeftCount;
            stripeStartRightCount = getRightEncoderCounts();

            stripeWidth = 0; // Reset stripeWidth
        }
    }
    else // IR sensor no longer detects a white stripe
    {
        if (isWhiteStripe) // Only calculate when leaving a white stripe
        {
            isWhiteStripe = 0;

            // Cache encoder counts at the end of the stripe
            int stripeEndLeftCount = currentLeftCount;
            int stripeEndRightCount = getRightEncoderCounts();

            // Calculate the encoder counts for the stripe
            int leftStripeCount = stripeEndLeftCount - stripeStartLeftCount;
            int rightStripeCount = stripeEndRightCount - stripeStartRightCount;

            // Calculate the average stripe width
            stripeWidth = (leftStripeCount + rightStripeCount) / 2;

            // Classify the stripe and store it in the barcode array
            stripeValue = -1; // Default invalid value
            if (stripeWidth > 270)
            {
                stripeValue = 1; // STRIPE_1
            }
            else if (stripeWidth > 100 && stripeWidth <= 270)
            {
                stripeValue = 0; // STRIPE_0
            }

            if (stripeValue != -1 && barcodeSize < MAX_BARCODE_BITS)
            {
                // Store the stripe value in the fixed-size array
                barcodeData[barcodeSize++] = stripeValue;

                // Update the current stripe value for Live Expressions
                currentStripeValue = stripeValue;
            }
        }
    }

    // Check if the barcode has ended
    if (((currentLeftCount - lastDetectionLeftCount) > BARCODE_END_THRESHOLD) && barcodeDetected == 1)
    {
        // Set the barcode complete flag for Live Expressions
        barcodeComplete = 1;
    }
}
void gotowardwall_encoders()
{
    frontwall_Distance = 0;
    if (ultr_int2 == 0)
    {
        resetEncoders();
        for (int i = 0; i < 10; i++)
        {
            frontwall_Distance = Ultrasonic_GetDistance(1);
            delay_ms(50);
        }
        ultr_int2 = 1;
    }
    if (frontwall_Distance >= 110)
    {
        distance = (frontwall_Distance - 110) * 6.2; // Adjust the distance as needed
        wall_follow_with_ultrasonic_encoders(140, 90, 2.7, 0.015, 0.015, distance);
        enc_drive_decel(62, 75, 60);
        m_stopLR();
        delay_ms(50);
        turn_Right_90_LR();
    }
    else
    {
        distance = (110 - frontwall_Distance) * 6.2; // Adjust the distance as needed
        enc_driveR_T(distance, 80, 60);              // Move forward until the distance is less than 110 cm
        m_stopLR();
        delay_ms(50);
        turn_Right_90_LR();
    }
}
int calculateDecimalValue()
{
    decimalValue = 0;

    // Iterate through the barcodeData array in reverse (LSB is at index 0)
    for (int i = 0; i < barcodeSize; i++)
    {
        decimalValue += barcodeData[i] * (1 << i); // Multiply bit by 2^i
    }

    return decimalValue;
}

Task3SubState currentTask3SubState = STATE_enter_from_ramp_entry;

void task3StateHandler()
{
    switch (currentTask3SubState)
    {
    case STATE_enter_from_ramp_entry:
        enc_drive_fast_to_target(6000, 150, 85);
        currentTask3SubState = STATE_enter_from_ramp;
        break;
    case STATE_enter_from_ramp:
        if (ultr_int2 == 0)
        {
            resetEncoders();
            for (int i = 0; i < 10; i++)
            {
                int frontwall_Distance = Ultrasonic_GetDistance(1);
                delay_ms(50);
            }
            ultr_int2 = 1;
        }
        enc_drive_with_ultrasonic_check_85(55, 1);
        if (wall == 1)
        {
            wall == 0;
            m_stopLR();
            delay_ms(30);
            resetEncoders();
            enc_driveR_T(61, 65, 60);
            turn_Right_90_LR_Controlled(825, 80);
            m_stopLR();
            delay_ms(50);
            resetEncoders();
            currentTask3SubState = STATE_2nd_wall;
        }
        break;
    case STATE_2nd_wall:
        display_TASK(3);
        ultr_int2 = 0;
        wall_follow_with_ultrasonic_encoders(140, 90, 2.7, 0.015, 0.015, 1000);
        enc_drive_decel(400, 80, 55);
        m_stopLR();
        delay_ms(50);
        gotowardwall_encoders();
        m_stopLR();
        wall_follow_with_ultrasonic_encoders(215, 90, 2.75, 0.018, 0.015, 1550);
        currentTask3SubState = STATE_to_pad;
        break;
    case STATE_to_pad:
        enc_drive2(90);
        if (IR[3] == 1)
        {
            enc_drive_decel(350, 90, 55);
            m_stopLR();
            delay_ms(50);
            setTCAChannel(0);
            delay_ms(10);
            for (int i = 0; i < 10; i++)
            {
                Bottom_C_PAD_Sensor();
                delay_ms(20);
            }
            if (bottomColor == COLOR_RED)
            {
                TSK3_first_PAD = 0;
            }
            else
            {
                delay_ms(80);
                TSK3_first_PAD = 1;
            }

            resetEncoders();
            enc_driveR_T(560, 65, 50);
            m_stopLR();
            delay_ms(50);
            turn_90_degreesL1();
            m_stopLR();
            delay_ms(50);
            enc_driveR_T(930, 90, 50);
            m_stopLR();
            delay_ms(80);
            enc_drive_accel(200, 200, 50);
            resetEncoders();
            currentTask3SubState = STATE_BarCode;
        }
        else
        {
            enc_drive2(100);
        }
        break;
    case STATE_BarCode:
        enc_drive_fast();
        detectBarCode();
        if (barcodeComplete == 1)
        {
            // Calculate the decimal value from the barcode data
            decimalValue = calculateDecimalValue();
            display_barcode_decimal(decimalValue);

            if (decimalValue % 2 == 0)
            {
                // even
                Y_balls_to_RED = 1;
            }
            else
            {
                Y_balls_to_BLUE = 1;
            }

            // Reset the barcode complete flag for the next barcode
            barcodeComplete = 0;

            // Stop the robot
            enc_drive_decel(1300, 130, 80);

            barcodeComplete = 2;
            // Update the decimal value for Live Expressions
            currentStripeValue = decimalValue; // Use this variable for debugging
        }
        if (barcodeComplete == 2)
        {
            currentTask3SubState = STATE_line_detect;
        }
        break;
    case STATE_line_detect:
        enc_drive2(80);
        if (IR[3] == 1)
        {
            enc_drive_decel(250, 80, 50);
            m_stopLR();
            display_barcode_decimal(decimalValue);
            delay_ms(50);
            turn_Left_90_LR(); // Turn left 90 degrees to drop balls
            m_stopLR();
            delay_ms(20);
            enc_driveR_T(820, 80, 60);
            m_stopLR();
            delay_ms(50);
            currentTask3SubState = STATE_Drop_balls;
        }
        else
        {
            enc_drive2(80);
        }
        break;
    case STATE_Drop_balls:
        if (Y_balls_to_RED == 1 && TSK3_first_PAD == 0)
        {
            // Drop white balls on the blue pad
            other_ball = 0;
            open_white_door();
            TankLift();
            delay_ms(100);
            close_white_door();
            TankLower();
        }
        else if (Y_balls_to_RED == 1 && TSK3_first_PAD == 1)
        {
            // Drop yellow balls on the red pad
            other_ball = 1;
            open_yellow_door();
            TankLift();
            delay_ms(100);
            close_yellow_door();
            TankLower();
        }
        else if (Y_balls_to_BLUE == 1 && TSK3_first_PAD == 0)
        {
            // Drop yellow balls on the blue pad
            other_ball = 1;
            open_yellow_door();
            TankLift();
            delay_ms(100);
            close_yellow_door();
            TankLower();
        }
        else
        {
            // Drop white balls on the red pad
            other_ball = 0;
            open_white_door();
            TankLift();
            delay_ms(100);
            close_white_door();
            TankLower();
        }
        currentTask3SubState = STATE_goto_next_PAD;
        break;
    case STATE_goto_next_PAD:
        enc_drive_T(820, 80, 60); // reposition
        m_stopLR();
        delay_ms(50);
        turn_Right_90_LR(); // Turn right 90 degrees go reverse
        m_stopLR();
        delay_ms(50);
        enc_driveR_T(3520, 130, 60); // go reverse
        m_stopLR();
        delay_ms(20);
        turn_Left_90_LR(); // Turn left 90 degrees to drop balls
        m_stopLR();
        delay_ms(20);
        enc_driveR_T(820, 80, 60);
        m_stopLR();
        delay_ms(50);
        currentTask3SubState = STATE_Drop_balls_second_PAD;
        break;
    case STATE_Drop_balls_second_PAD:
        if (other_ball == 0)
        {
            // Drop Yellow Balls
            open_yellow_door();
            TankLift();
            delay_ms(100);
            close_yellow_door();
            TankLower();
        }
        else
        {
            // Drop White Balls
            open_white_door();
            TankLift();
            delay_ms(100);
            close_white_door();
            TankLower();
        }
        currentTask3SubState = reposition;
        break;
    case reposition:
        enc_drive_T(820, 80, 60); // reposition
        m_stopLR();
        delay_ms(50);
        turn_Right_90_LR(); // Turn right 90 degrees to reposition to go front
        m_stopLR();
        delay_ms(50);
        enc_drive_accel(400, 180, 50);
        enc_drive_fast_to_target(2400, 180, 60); // go front
        m_stopLR();
        delay_ms(50);
        turn_Left_90_LR();
        m_stopLR();
        delay_ms(50);
        currentTask3SubState = STATE_Detect_Next_Line;
        break;
    case STATE_Detect_Next_Line:
        wall_follow_with_ultrasonic(100, 80, 2.85, 0, 0.02);
        resetEncoders();
        enc_drive_decel(750, 80, 65);
        m_stopLR();
        delay_ms(80);
        resetEncoders();
        turn_Right_90_LR();
        delay_ms(50);
        currentMainState = TASK_4;
        break;
    }
}
