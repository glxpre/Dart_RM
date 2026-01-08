//
// Created by 18723 on 2025/8/1.
//
#ifndef SENTRY_GIMBAL_TIMER_H
#define SENTRY_GIMBAL_TIMER_H

#include "bsp_dwt.h"
#define MY_TIMER_COUNT 4

typedef enum{
    TIMERMOD_DICK,
    TIMERMOD_OTHER
} Timer_Mod_t;

typedef struct MyTimer_t{
    Timer_Mod_t mod;
    float last_time;
    float delta_time;
    float delta_time_set;
    uint8_t flag;

    void (*updata)(struct MyTimer_t*);
} MyTimerInstance_t;

MyTimerInstance_t* TimerInit(float set_time_ms, Timer_Mod_t select_mod);
void TimerTask();
uint8_t TimerFlagGet(MyTimerInstance_t*);


#endif //SENTRY_GIMBAL_TIMER_H
