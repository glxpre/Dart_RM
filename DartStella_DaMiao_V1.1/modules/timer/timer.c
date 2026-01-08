//
// Created by 18723 on 2025/8/1.
//
#include <string.h>
#include "timer.h"

static uint8_t idx = 0;
static MyTimerInstance_t *my_timer_instance[MY_TIMER_COUNT] = {NULL};

/**
 * @brief 每满足间隔时间便会置1，置1时再引用便重新开始
 * @param timer
 * @return
 */
static void UpdateTime(MyTimerInstance_t* timer){
    float current_time = DWT_GetTimeline_ms();
    timer->delta_time = current_time - timer->last_time;
//    if(timer->flag != 0) timer->flag = 0;
    if(timer->delta_time > timer->delta_time_set && timer->flag == 0) {
        //超过计时时间，清零并使标志置一
        timer->last_time = current_time;
        timer->flag = 1;
    }
}



/**
 * @brief 每引用一次加一
 * @param timer
 * @return
 */
void updata_cnt(MyTimerInstance_t* timer){

}

MyTimerInstance_t* TimerInit(float set_time_ms, Timer_Mod_t select_mod){
    MyTimerInstance_t *timer_instance = (MyTimerInstance_t *) malloc(sizeof(MyTimerInstance_t));
    memset(timer_instance, 0, sizeof(MyTimerInstance_t));

    if(timer_instance != NULL){
        timer_instance->delta_time_set = set_time_ms;
        timer_instance->mod = (select_mod == 1) ? TIMERMOD_OTHER : TIMERMOD_DICK; //非1皆为默认0
        timer_instance->updata = (select_mod == 1) ? updata_cnt : UpdateTime;

    }
    else {
        //错误处理
    }

    my_timer_instance[idx++] = timer_instance;
    return timer_instance;
}

void TimerTask(){
    MyTimerInstance_t *timer;
    for(uint8_t i = 0; i<idx; i++){
        timer = my_timer_instance[i];
        timer->updata(timer);
    }

}

uint8_t TimerFlagGet(MyTimerInstance_t *timer){
    if(!timer->flag) return 0;
    timer->flag = 0;
    return 1;
}
