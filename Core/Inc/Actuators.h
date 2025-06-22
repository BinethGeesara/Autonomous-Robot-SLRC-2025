#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "PCA9685.h" // Include the PCA9685 library
#include "Timer_Delay.h" // Include delay functions

// Function prototypes
void Actuator_Init();
void TankLift();
void TankLower();
/// @brief ////////
void WhiteTnk();
void W_Nutral();
void YellowTnk();
void Y_Nutral();
/// @brief ////////
void open_white_door();
void close_white_door();
void open_yellow_door();
void close_yellow_door();
//////////////////
void Arm_Down();
void Arm_Up();
void Arm_mid();
void BOX_Arm_Down();
void BOX_Arm_Up();
void Gripper_Open();
void Gripper_Close();
void Gripper_Normal();
void BOX_Gripper_Open();
void BOX_Gripper_Close();
//////////////////////
void ball_pick_Y();
void ball_pick_W();
////////////////////
void water_pump_up();
void water_pump_down();
void water_pump_mid_to_down();
void water_pump_down_to_mid();
void water_pump_mid();
void pump_1_on();
void pump_1_off();
void pump_2_on();
void pump_2_off();
#endif // ACTUATORS_H