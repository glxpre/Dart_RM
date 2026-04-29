//
// Created by 17946 on 26-1-4.
//

#ifndef AUTO_ROUTINE_H
#define AUTO_ROUTiNE_H
#include "robot_def.h"


typedef enum {
    DEBUG_STATE_INDEX,
    SLIP_DOWN_STATE_INDEX,
    SLIP_UP_STATE_INDEX,
    SCREW_ADJUST_FORCE_STATE_INDEX,
    TRIGGER_ON_STATE_INDEX,
    RELOAD_STATE_INDEX,
    WAIT_TRIGGER_STATE_INDEX
}STATE;

Shoot_Ctrl_Cmd_s *get_auto_shoot_send(void);
void AutoAim_Init();
void AutoAim_Task();

#endif //AUTO_ROUTINE_H
