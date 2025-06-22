#ifndef TASK_5_H
#define TASK_5_H

typedef enum
{
  STATE_display,
  STATE_Line_Detect,
  STATE_right,
  STATE_left,
  STATE_pad2,
  STATE_3
} Task5SubState;

void task5StateHandler();

#endif // TASK_5_H