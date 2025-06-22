#ifndef TASK_6_H
#define TASK_6_H

void task6StateHandler();

typedef enum
{
    STATE_Display,
    STATE_entering,
    STATE_Plant_line_detection,
    STATE_2ndplant_line_detected,
    STATE_WaterTank_detected,
    STATE_PlantTLine_detection,
    STATE_Plant_T_detection,
    STATE_Plant_detection,
    STATE_Plant_Check,
    STATE_Done,
    STATE_TeST,
    STATE_Arm_Down
} Task6SubState;

#endif // TASK_6_H