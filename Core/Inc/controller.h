#ifndef CONTROLLER_H
#define CONTROLLER_H

void enc_drive();
void enc_drive_fast();
void enc_drive_fast_to_target();
void enc_drive_decel();
void enc_driveR_decel();
void enc_drive_accel();
void enc_drive_T();
void enc_drive2();
void enc_driveR();
void enc_driveR_T();

void align_to_line();
void angle_error_calc();
void turn_Left_align();

void line_follow_with_ultrasonic_check();
void enc_drive_with_ultrasonic_check();
void enc_drive_with_ultrasonic_check_85();
void enc_drive_with_ultrasonic_Target();

void wall_follow_with_ultrasonic2(int target_distance, int base_speed, float Kp, float Ki, float Kd);
void wall_follow_with_ultrasonic_encoders(int target_distance, int base_speed, float Kp, float Ki, float Kd, int target_encoder_counts);
void wall_follow_with_ultrasonic(int target_distance, int base_speed, float Kp, float Ki, float Kd);

void line_follow_to_target();
void line_follow();
void line_follow_NoJunctions();
void line_follow_Last_Column();
void turn_90_degreesR();
void turn_90_degreesR1();
void turn_90_degreesR1_reverse();
void turn_90_degreesR1_reverse_FOR_PAD();
void turn_90_degreesR_IR8();
void turn_90_degreesR_TS2();
void turn_90_degreesR_After_PadColor();

void turn_20_degrees_L();
void turn_20_degrees_R();
void turn_90_degreesL();
void turn_90_degreesL1_reverse_FROM_PAD();
void turn_90_degreesL_fromLastColumn();

void turn_right_45_degrees_right_Wh();
void turn_left_45_degrees_right_Wh();
///////////////
void turn_Right_45_TASK2_LR();
void turn_Left_45_TASK2_LR();
void turn_Right_45_TASK2_R1();
void turn_90_degreesL1();

////////////
void turn_Right_45_LR();
void turn_Left_180_LR();
void turn_Right_180_LR();
void turn_Right_90_LR();
void turn_Left_90_LR();
/////////////////////
void turn_Right_90_LR_Controlled();
void turn_90_degreesR1_Controlled();
////////task4////////
void T4_turn_Right_180_LR();
void T4_turn_Right_90_LR();
void T4_turn_Left_180_LR();
void T4_turn_Left_90_LR();
void T4_turn_Right_45_LR();
void T4_turn_Right_90_LR_Controlled();
#define BASE_SPEED 90

#endif // CONTROLLER_H
