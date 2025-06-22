#include "Actuators.h"
#include "i2c_MUX.h"

/*flap-1
door - 2
TankLift - 3
Gripper_Arm - 4
Gripper - 5*/

// Initialize the actuators
void Actuator_Init()
{
    setTCAChannel(2);
    PCA9685_Init(50);
    PCA9685_SetServoAngle(1, 110);
    PCA9685_SetServoAngle(2, 90);
    PCA9685_SetServoAngle(3, 100);
    PCA9685_SetServoAngle(4, 60); // 170
    PCA9685_SetServoAngle(5, 170);
    PCA9685_SetServoAngle(14, 0); // 170
}

void TankLift()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(3, 100, 60, 10);
    delay_ms(800);
}
void TankLower()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(3, 60, 100, 10);
    delay_ms(300);
}
/// @brief ////////////////////////////////
void WhiteTnk()
{
    setTCAChannel(2);
    PCA9685_SetServoAngle(1, 60);
    // PCA9685_SetServoAngleSlow(1,100,50,5);
}

void W_Nutral()
{
    setTCAChannel(2);
    PCA9685_SetServoAngle(1, 110);
    // PCA9685_SetServoAngleSlow(1,50,100,5);
}

void YellowTnk()
{
    setTCAChannel(2);
    PCA9685_SetServoAngle(1, 143);
    // PCA9685_SetServoAngleSlow(1,100,143,5);
}
void Y_Nutral()
{
    setTCAChannel(2);
    PCA9685_SetServoAngle(1, 110);
    // PCA9685_SetServoAngleSlow(1,143,100,5);
}
/// @brief //////////////////////////////////
void open_white_door()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(1, 100, 115, 5);
    PCA9685_SetServoAngleSlow(2, 90, 180, 5);
    delay_ms(400);
}
void close_white_door()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(1, 115, 100, 5);
    PCA9685_SetServoAngleSlow(2, 180, 90, 5);
    delay_ms(400);
}
void open_yellow_door()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(1, 100, 85, 5);
    PCA9685_SetServoAngleSlow(2, 90, 0, 5);
    delay_ms(400);
}
void close_yellow_door()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(1, 85, 100, 5);
    PCA9685_SetServoAngleSlow(2, 0, 90, 5);
    delay_ms(400);
}

/////////////////////////////////////

void Arm_Down()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 55, 176, 10);
    delay_ms(500);
}
void BOX_Arm_Down()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 55, 178, 10);
    delay_ms(500);
}
void BOX_Arm_Up()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 178, 55, 10);
    delay_ms(500);
}
void Arm_Up()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 176, 40, 10);
}

void Arm_mid()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 40, 55, 10);
}
void Gripper_Open()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(5, 170, 130, 5);
}

void Gripper_Close()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(5, 130, 170, 5);
    delay_ms(500);
}

void Gripper_Normal()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(5, 110, 160, 5);
}
void BOX_Gripper_Open()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(5, 170, 125, 5);
}
void BOX_Gripper_Close()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(5, 125, 170, 5);
    delay_ms(500);
}
/*WhiteTnk();
    Gripper_Open();
    Arm_Down();
    Gripper_Close();
    Arm_Up();
    Gripper_Open();
    delay_ms(5000);*/
////////////////////
void ball_pick_Y()
{
    YellowTnk();
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 60, 80, 10);
    delay_ms(500);
    Gripper_Open();
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 80, 174, 10);
    Gripper_Close();
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 174, 40, 10);
    Gripper_Open();
    delay_ms(600);
    Gripper_Close();
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 40, 60, 10);
    Y_Nutral();
}
void ball_pick_W()
{
    WhiteTnk();
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 60, 80, 10);
    delay_ms(500);
    Gripper_Open();
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 80, 174, 10);
    Gripper_Close();
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 174, 40, 10);
    Gripper_Open();
    delay_ms(600);
    Gripper_Close();
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(4, 40, 60, 10);
    W_Nutral();
}
////////////////////
void water_pump_up()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(14, 108, 0, 0);
    delay_ms(1000);
}
void water_pump_mid()
{
    setTCAChannel(2);
    delay_ms(1000);
    PCA9685_SetServoAngle(14, 10);
    delay_ms(800);
}
void water_pump_down()
{
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(14, 0, 108, 0);
    delay_ms(1000);
}
void water_pump_mid_to_down(){
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(14, 60, 110, 0);
    delay_ms(1000);
}
void water_pump_down_to_mid(){
    setTCAChannel(2);
    PCA9685_SetServoAngleSlow(14, 110, 60, 0);
    delay_ms(1000);
}
//////////////////////////////////////
void pump_1_on()
{
    HAL_GPIO_WritePin(GPIOB, PUMP_1_Pin, GPIO_PIN_SET); // Turn on PUMP_1
}
void pump_1_off()
{
    HAL_GPIO_WritePin(GPIOB, PUMP_1_Pin, GPIO_PIN_RESET); // Turn off PUMP_1
}
void pump_2_on()
{
    HAL_GPIO_WritePin(PUMP_2_GPIO_Port, PUMP_2_Pin, GPIO_PIN_SET);
}
void pump_2_off()
{
    HAL_GPIO_WritePin(PUMP_2_GPIO_Port, PUMP_2_Pin, GPIO_PIN_RESET);
}
