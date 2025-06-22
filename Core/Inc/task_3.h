#ifndef TASK_3_H
#define TASK_3_H

typedef enum {
    STATE_TASK3,
    STATE_enter_from_ramp_entry,
    STATE_enter_from_ramp,
    STATE_2nd_wall,
    STATE_to_pad,
    STATE_BarCode,
    STATE_line_detect,
    STATE_BarCode_1,
    STATE_BarCode_2,
    STATE_BarCode_3,
    STATE_Drop_balls,
    STATE_goto_next_PAD,
    STATE_Drop_balls_second_PAD,
    reposition,
    STATE_Detect_Next_Line
} Task3SubState;

void task3StateHandler();

#endif