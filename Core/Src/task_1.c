#include "task_1.h"
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
extern int junction_count;
extern int junction_detected;
extern int start_caliberate;
extern int start_task_1;
extern int start_task_2;
extern int start_task_3;
extern int start_task_4;
extern int start_task_5;
extern int start_task_6;
int junction_detected = 0;
int junction_count = 0;
int junc_count_lastColumn_verify = 0;
int junc_count_lastColumn = 0;
int columns_visited = 0;
int state_init_processed = 0;
int force_junc = 0;
extern float correction;
extern int difference_;
extern int Button1;
extern int Button2;

int RED_PAD = 0;
int BLUE_PAD = 0;
int next_pad_blue = 0;
int next_pad_red = 0;
int task_done = 0;
int brake_for_linefollow = 0;
extern int cylinder_Distance;
extern int cylinder_detected;
int Distance1;
int Needtostop = 0;
unsigned long lastUltrasonicTime = 0;
const unsigned long ultrasonicInterval = 50; // 50ms delay

Task1SubState currentTask1SubState = STATE_DisplayMenu;

void turn_Right_90_LR_IR8()
{
  resetEncoders();
  const int maxSpeed = 95; // Maximum motor speed
  const int minSpeed = 66; // Minimum speed to overcome friction
  const int accelSteps = 300;
  const int target = 825;              // Counts to accelerate
  const int decelStart = target - 480; // When to start decelerating

  int currentSpeed = minSpeed; // Starting speed

  while (1)
  {
    int left_counts = abs(getLeftEncoderCounts());
    int right_counts = abs(getRightEncoderCounts());
    int avg_counts = (left_counts + right_counts) / 2;

    if (IR[8] == 1)
    {
      junction_count = 1;
      force_junc = 1;
      // No break here, so the loop continues
    }
    // Acceleration phase
    if (avg_counts < accelSteps)
    {
      currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
    }
    // Deceleration phase
    else if (avg_counts > decelStart)
    {
      int remaining = target - avg_counts;
      if (remaining > 0)
      {
        currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
      }
      else
      {
        currentSpeed = 0;
      }
    }
    // Constant speed phase
    else
    {
      currentSpeed = maxSpeed;
    }

    // Balance correction
    int error = left_counts - right_counts;
    int correction = error * 1;

    // Apply speeds with minimum guaranteed
    int speedL = -(currentSpeed + correction);
    int speedR = currentSpeed - correction;

    // Ensure minimum speed is maintained
    if (abs(speedL) < minSpeed)
      speedL = speedL < 0 ? -minSpeed : minSpeed;
    if (abs(speedR) < minSpeed)
      speedR = speedR < 0 ? -minSpeed : minSpeed;

    setMotorSpeedL(speedL);
    setMotorSpeedR(speedR);

    // Exit condition
    if (left_counts >= target && right_counts >= target)
    {
      break;
    }
  }

  m_stopLR();
  delay_ms(50);
  resetEncoders();
}

void task1StateHandler()
{
  switch (currentTask1SubState)
  {
  case STATE_TESTING:
    enc_driveR_T(1000, 70, 65);
    delay_ms(1000);
    currentTask1SubState = STATE_TESTING;
    break;
  case STATE_TESTING2:
    enc_driveR(70);
    break;
  case STATE_INIT:
    display_TASK(1);
    // Check if STATE_INIT has already been processed
    if (!state_init_processed)
    {
      columns_visited++;        // Increment only once per transition
      state_init_processed = 1; // Mark the state as processed
    }
    junction_count = 0;
    enc_drive();

    if (IR[7] == 1 && IR[8] == 1)
    {
      // Decelerate for 700 encoder counts
      enc_drive_decel(750, 80, 65);
      m_stopLR();
      delay_ms(80);
      resetEncoders();
      turn_Right_90_LR_IR8();

      // STATE_linefollow_lastColumn
      if (force_junc == 1)
      {
        force_junc = 0;
        currentTask1SubState = STATE_JUNC_Detected;
      }
      else
      {
        currentTask1SubState = STATE_TASK1;
      }
    }
    else if (columns_visited == 6)
    {
      currentTask1SubState = STATE_SWITCH_L_FOLLOW_LASTCOLUMN_2;
    }
    // Reset the flag when exiting the state
    if (currentTask1SubState != STATE_INIT)
    {
      state_init_processed = 0;
    }
    break;
  case STATE_Align_to_lastColumn:

    enc_drive();

    break;
  case STATE_SWITCH_L_FOLLOW_LASTCOLUMN:
    enc_drive();
    if (getLeftEncoderCounts() > 100 && getRightEncoderCounts() > 100)
    {
      m_stopLR();
      delay_ms(100);
      currentTask1SubState = STATE_SWITCH_L_FOLLOW_LASTCOLUMN_2;
    }
    else
    {
      enc_drive();
    }
  case STATE_SWITCH_L_FOLLOW_LASTCOLUMN_2:
    enc_drive();
    if (IR[7] == 1 && IR[8] == 1)
    {
      // Decelerate for 700 encoder counts
      enc_drive_decel(124, 85, 60);
      resetEncoders();
      currentMainState = TASK_2;
    }
    else
    {
      enc_drive();
    }
    break;
  case STATE_TASK1:
    line_follow();
    if (junction_detected == 1)
    {
      brake_for_linefollow = 1;
      junction_count++;
      currentTask1SubState = STATE_JUNC_Detected;
    }
    else
    {
      line_follow();
    }
    break;
  case STATE_linefollow_lastColumn:
    line_follow_Last_Column();
    if ((IR[7] == 1) && (IR[2] == 1 || IR[3] == 1 || IR[4] == 1))
    {
      if (junc_count_lastColumn_verify == 0)
      {
        m_stopLR();
        junc_count_lastColumn++;
        junc_count_lastColumn_verify = 1;
        resetEncoders();
        while (getLeftEncoderCounts() < 740 && getRightEncoderCounts() < 740)
        {
          line_follow();
        }
        if (junc_count_lastColumn == 2)
        {
          currentTask1SubState = STATE_TurnTO_Find_PAD;
        }
      }
    }
    else
    {
      junc_count_lastColumn_verify = 0;
      line_follow_Last_Column();
    }
    break;
  case STATE_TurnTO_Find_PAD:
    m_stopLR();
    delay_ms(200);
    turn_90_degreesL_fromLastColumn();
    delay_ms(100);
    currentTask1SubState = STATE_Find_PAD;

    break;
  case STATE_Find_PAD:
    enc_drive();
    if (IR[1] == 1 || IR[2] == 1 || IR[3] == 1 || IR[4] == 1 || IR[5] == 1)
    {
      m_stopLR();
      delay_ms(100);
      resetEncoders();
      currentTask1SubState = STATE_Pad_Colour_Detection;
    }
    else
    {
      enc_drive();
    }
    break;
  case STATE_Pad_Colour_Detection:
    enc_drive();
    if (getLeftEncoderCounts() > 250 && getRightEncoderCounts() > 250)
    {
      m_stopLR();
      delay_ms(100);
      turn_20_degrees_L();
      delay_ms(100);

      setTCAChannel(0);
      delay_ms(10);
      for (int i = 0; i < 10; i++)
      {
        Bottom_C_PAD_Sensor();
        delay_ms(20);
      }
      if (bottomColor == COLOR_RED)
      {
        delay_ms(800);
        RED_PAD = 1;
        currentTask1SubState = STATE_reposition_after_pad_color;
      }
      else
      {
        delay_ms(800);
        BLUE_PAD = 1;
        currentTask1SubState = STATE_reposition_after_pad_color;
      }
    }
    else
    {
      enc_drive();
    }
    break;
  case STATE_reposition_after_pad_color:
    turn_20_degrees_R();
    delay_ms(100);
    resetEncoders();
    currentTask1SubState = STATE_reposition_after_pad_color_1;

    break;
  case STATE_reposition_after_pad_color_1:
    enc_driveR(70);
    if (getLeftEncoderCounts() < -110 && getRightEncoderCounts() < -110)
    {
      m_stopLR();
      delay_ms(100);
      turn_90_degreesR_After_PadColor();
      delay_ms(100);
      currentTask1SubState = STATE_reposition_after_pad_color_2;
    }
    else
    {
      enc_driveR(70);
    }
    break;
  case STATE_reposition_after_pad_color_2:
    enc_driveR(70);
    if (getLeftEncoderCounts() < -390 && getRightEncoderCounts() < -390)
    {
      m_stopLR();
      delay_ms(100);
      turn_90_degreesR1_reverse_FOR_PAD();
      delay_ms(100);
      resetEncoders();
      currentTask1SubState = STATE_reposition_after_pad_color_3;
    }
    else
    {
      enc_driveR(70);
    }
    break;
  case STATE_reposition_after_pad_color_3:
    enc_driveR(70);
    if (getLeftEncoderCounts() < -450 && getRightEncoderCounts() < -450)
    {
      m_stopLR();
      delay_ms(800);
      if (RED_PAD == 1)
      {
        open_yellow_door();
        TankLift();
        delay_ms(1000);
        close_yellow_door();
        TankLower();
        next_pad_blue = 1;
        delay_ms(1000);
        resetEncoders();
        currentTask1SubState = STATE_reposition_for_2nd_pad;
      }
      else if (BLUE_PAD == 1)
      {
        open_white_door();
        TankLift();
        delay_ms(1000);
        close_white_door();
        TankLower();
        next_pad_red = 1;
        delay_ms(1000);
        resetEncoders();
        currentTask1SubState = STATE_reposition_for_2nd_pad;
      }
    }
    else
    {
      enc_driveR(70);
    }
    break;
  case STATE_reposition_for_2nd_pad:
    enc_drive();
    if (getLeftEncoderCounts() > 448 && getRightEncoderCounts() > 448)
    {
      turn_90_degreesL1_reverse_FROM_PAD();
      delay_ms(100);
      resetEncoders();
      if (task_done == 1)
      {
        currentTask1SubState = STATE_TASK_DONE;
      }
      else
      {
        currentTask1SubState = STATE_reposition_for_2nd_pad_1;
      }
    }
    else
    {
      enc_drive();
    }

    break;
  case STATE_reposition_for_2nd_pad_1:
    enc_drive();
    if (getLeftEncoderCounts() > 1788 && getRightEncoderCounts() > 1788)
    {
      m_stopLR();
      delay_ms(100);
      turn_90_degreesR1_reverse_FOR_PAD();
      delay_ms(800);
      resetEncoders();
      currentTask1SubState = STATE_reposition_for_2nd_pad_2;
    }
    else
    {
      enc_drive();
    }
    break;
  case STATE_reposition_for_2nd_pad_2:
    enc_driveR(70);
    if (getLeftEncoderCounts() < -500 && getRightEncoderCounts() < -500)
    {
      m_stopLR();
      delay_ms(800);
      if (next_pad_red == 1)
      {
        open_yellow_door();
        TankLift();
        delay_ms(1000);
        close_yellow_door();
        TankLower();
        RED_PAD = 0;
        delay_ms(1000);
        resetEncoders();
        task_done = 1;
        currentTask1SubState = STATE_reposition_for_2nd_pad;
      }
      else if (next_pad_blue == 1)
      {
        open_white_door();
        TankLift();
        delay_ms(100);
        close_white_door();
        TankLower();
        BLUE_PAD = 0;
        delay_ms(1000);
        resetEncoders();
        task_done = 1;
        currentTask1SubState = STATE_reposition_for_2nd_pad;
      }
    }
    else
    {
      enc_driveR(70);
    }
    break;
  case STATE_JUNC_Detected:
    cylinder_detected = 0;
    if (brake_for_linefollow == 1)
    {
      brake_for_linefollow = 0;
      enc_drive_decel(100, 80, 65);
      m_stopLR();
      delay_ms(30);
    }
    resetEncoders();
    for (int i = 0; i < 10; i++)
    {
      Distance1 = Ultrasonic_GetDistance(0);
      delay_ms(50);
    }
    line_follow_with_ultrasonic_check(800);
    enc_drive_decel(250, 80, 65);
    m_stopLR();
    delay_ms(10);
    setTCAChannel(0);
    delay_ms(10);
    for (int i = 0; i < 10; i++)
    {
      Bottom_C_PAD_Sensor();
      delay_ms(5);
    }
    if (bottomColor == COLOR_GREEN)
    {
      currentTask1SubState = STATE_G_DETECTED;
      break; // Add break statement here
    }
    else
    {
      if (junction_count == 3)
      {
        currentTask1SubState = STATE_Grid_reverse;
        break;
      }
      resetEncoders();
      currentTask1SubState = STATE_TASK1;
    }
    break;
  case STATE_G_DETECTED:
    if (cylinder_detected == 1)
    {
      resetEncoders();
      cylinder_detected = 0;
      currentTask1SubState = STATE_Obj_C_detection;
      break;
      // currentState = STATE_Grid_reverse;
    }
    else
    {
      currentTask1SubState = STATE_TASK1;
    }
    break;
  case STATE_Obj_C_detection:
    enc_driveR(70);
    if (IR[7] == 1 && IR[8] == 1)
    {
      enc_driveR_decel(56, 70, 60);
      m_stopLR();
      delay_ms(100);
      enc_drive_T(180, 80, 65);
      m_stopLR();
      delay_ms(10);
      turn_left_45_degrees_right_Wh();
      delay_ms(10);
      resetEncoders();
      currentTask1SubState = STATE_Ball_Detection;
    }
    else
    {
      enc_driveR(70);
    }
    break;
  case STATE_Ball_Detection:
    setTCAChannel(3);
    delay_ms(10);
    Top_C_BALL_Sensor();
    delay_ms(100);
    if (topColor == COLOR_ORANGE)
    {
      enc_driveR_T(31, 66, 60);
      YellowTnk();
      Gripper_Open();
      Arm_Down();
      Gripper_Close();
      Arm_Up();
      Gripper_Open();
      delay_ms(600);
      Gripper_Close();
      Arm_mid();
      delay_ms(500);
      Y_Nutral();
      resetEncoders();
      currentTask1SubState = STATE_Ball_Picked;
    }
    else if (topColor == COLOR_WHITE)
    {
      enc_driveR_T(31, 66, 60);
      WhiteTnk();
      Gripper_Open();
      Arm_Down();
      Gripper_Close();
      Arm_Up();
      Gripper_Open();
      delay_ms(600);
      Gripper_Close();
      Arm_mid();
      delay_ms(500);
      W_Nutral();
      resetEncoders();
      currentTask1SubState = STATE_Ball_Picked;
    }
    break;
  case STATE_Ball_Picked:
    enc_driveR_T(144, 66, 60);
    turn_Right_45_LR();
    // enc_driveR_T(22, 70, 60);
    currentTask1SubState = STATE_Grid_reverse;
    break;
  case STATE_Grid_reverse:

    if (junction_count == 3 || junction_count == 2)
    {
      junction_count--;
      turn_Left_180_LR();
      currentTask1SubState = STATE_GRID_R_LineFolow;
    }
    else
    {
      resetEncoders();
      currentTask1SubState = STATE_GRID_REPOSJ1;
    }
    break;

  case STATE_GRID_REPOSJ1:
    enc_driveR_T(260, 70, 65);
    m_stopLR(); // Stop the motors after reaching the target
    delay_ms(100);
    turn_Left_90_LR();
    currentTask1SubState = STATE_INIT;
    break;

  case STATE_GRID_R_LineFolow:
    line_follow();
    if ((IR[2] == 1 || IR[3] == 1 || IR[4] == 1 || IR[5] == 1 || IR[1] == 1) && (IR[7] == 1 && IR[8] == 1))
    {
      junction_count--;
      if (junction_count == 0)
      {
        resetEncoders();
        currentTask1SubState = STATE_GRID_REPOSJ2_3;
      }
      else
      {
        resetEncoders();
        while (getLeftEncoderCounts() < 500 && getRightEncoderCounts() < 500)
        {
          line_follow();
        }
      }
    }
    else
    {
      line_follow();
    }
    break;

  case STATE_GRID_REPOSJ2_3:
    while (getLeftEncoderCounts() < 750 && getRightEncoderCounts() < 750)
    {
      line_follow();
    }
    enc_drive_decel(100, 80, 65);
    m_stopLR(); // Stop the motors after reaching the target
    delay_ms(50);
    turn_90_degreesR1();
    currentTask1SubState = STATE_INIT; // Transition to the next state
    break;
  case STATE_TASK_DONE:
    enc_drive();
    if (getLeftEncoderCounts() > 560 && getRightEncoderCounts() > 560)
    {
      m_stopLR();
      delay_ms(30000);
    }
    else
    {
      enc_drive();
    }
    break;

  case STATE_DisplayMenu:
    UpdateOLEDMenu(); // Update the OLED menu
    if (start_task_1 == 1)
    {
      display_TASK(1);
      delay_ms(2000);
      currentTask1SubState = STATE_INIT; // Transition to Task 1
      start_task_1 = 0;                  // Reset the flag
    }
    else if (start_task_2 == 1)
    {
      display_TASK(2);
      delay_ms(2000);
      currentMainState = TASK_2; // Transition to Task 2
      start_task_2 = 0;          // Reset the flag
    }
    else if (start_task_3 == 1)
    {
      display_TASK(3);
      delay_ms(2000);
      currentMainState = TASK_3; // Transition to Task 3
      start_task_3 = 0;          // Reset the flag
    }
    else if (start_task_4 == 1)
    {
      display_TASK(4);
      delay_ms(2000);
      currentMainState = TASK_4; // Transition to Task 4
      start_task_4 = 0;          // Reset the flag
    }
    else if (start_task_5 == 1)
    {
      display_TASK(5);
      delay_ms(2000);
      currentMainState = TASK_5; // Transition to Task 5
      start_task_5 = 0;          // Reset the flag
    }
    else if (start_task_6 == 1)
    {
      display_TASK(6);
      delay_ms(2000);
      currentMainState = TASK_6; // Transition to Task 6
      start_task_6 = 0;          // Reset the flag
    }
    break;
  }
}
