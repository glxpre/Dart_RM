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
    WAIT_TRIGGER_STATE_INDEX,
    EMERGENCY_STATE_INDEX,
}STATE;

//遥控类型
typedef enum {
    AUTO_SHOOT_REQUEST_NONE,
    AUTO_SHOOT_REQUEST_REMOTE,
    AUTO_SHOOT_REQUEST_REFEREE,
} AutoShootRequestSource_e;

//舱门状态
typedef enum {
    Dart_station_close,
    Dart_station_open,
    Dart_station_active_close2open,
    Dart_station_active_open2close,
}Dart_station_state_enum;

void AutoAim_Init();
void AutoAim_Task();

#endif //AUTO_ROUTINE_H
