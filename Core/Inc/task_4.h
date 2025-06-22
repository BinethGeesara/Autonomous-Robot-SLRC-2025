#ifndef TASK_4_H
#define TASK_4_H

typedef enum {
    STATE_linefollow,
    STATE_0_ROW,
    STATE_1_ROW,
    STATE_2_ROW,
    STATE_TASK4,
    STATE_Task4_1,
    STATE_Task4_2,
    STATE_Task4_3,
    STATE_Task4_4,
    FinishTASK4,
    FinishToline
} Task4SubState;

void task4StateHandler();

#endif