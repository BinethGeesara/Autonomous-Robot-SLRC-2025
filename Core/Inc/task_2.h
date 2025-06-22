#ifndef TASK_2_H
#define TASK_2_H

typedef enum {
    STATE_TASK2,
    STATE_TASK2_A,
    STATE_TASK2_B,
    STATE_TASK2_A1,
    STATE_Ramp
} Task2SubState;

void task2StateHandler(void);

#endif