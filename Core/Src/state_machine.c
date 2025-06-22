#include "state_machine.h"
#include "task_1.h"
#include "task_2.h"
#include "task_3.h"
#include "task_4.h"
#include "task_5.h"
#include "task_6.h"


MainState currentMainState = TASK_1;

void updateStateMachine()
{
    switch (currentMainState)
    {
    case TASK_1:
        task1StateHandler();
        break;
    case TASK_2:
        task2StateHandler();
        break;
    case TASK_3:
        task3StateHandler();
        break;
    case TASK_4:
        task4StateHandler();
        break;
    case TASK_5:
        task5StateHandler();
        break;
    case TASK_6:
        task6StateHandler();
        break;
    }
}
