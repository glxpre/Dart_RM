//
// Created by 17946 on 26-1-4.
//

#ifndef AUTO_ROUTINE_H
#define AUTO_ROUTiNE_H
#include "robot_def.h"


Shoot_Ctrl_Cmd_s *get_auto_shoot_send(void);
void AutoAim_Init();
void AutoAim_Task();

#endif //AUTO_ROUTINE_H
