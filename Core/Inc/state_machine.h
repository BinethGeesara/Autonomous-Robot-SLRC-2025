#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

typedef enum {
    TASK_1,
    TASK_2,
    TASK_3,
    TASK_4,
    TASK_5,
    TASK_6
} MainState;

void updateStateMachine();

extern int Needtostop;
#endif
