#ifndef TASK_1_H
#define TASK_1_H

typedef enum {
    STATE_TESTING,
    STATE_TESTING2,
    STATE_INIT,
    STATE_G_DETECTED,
    STATE_Obj_C_detection,
    STATE_Ball_Detection,
    STATE_Ball_Picked,
    STATE_Grid_reverse,
    STATE_TASK1,
    STATE_Align_to_lastColumn,
    STATE_SWITCH_L_FOLLOW_LASTCOLUMN,
    STATE_SWITCH_L_FOLLOW_LASTCOLUMN_2,
    STATE_linefollow_lastColumn,
    STATE_TurnTO_Find_PAD,
    STATE_Find_PAD,
    STATE_Pad_Colour_Detection,
    STATE_reposition_after_pad_color,
    STATE_reposition_after_pad_color_1,
    STATE_reposition_after_pad_color_2,
    STATE_reposition_after_pad_color_3,
    STATE_reposition_for_2nd_pad,
    STATE_reposition_for_2nd_pad_1,
    STATE_reposition_for_2nd_pad_2,
    STATE_JUNC_Detected,
    STATE_GRID_REPOSJ1,
    STATE_GRID_REPOSJ2_3,
    STATE_GRID_R_LineFolow,
    STATE_TASK6,
    STATE_TASK_DONE,
    STATE_DisplayMenu,
    STATE_Caliberate
  } Task1SubState;
  
void task1StateHandler();

#endif