//
// Created by 17946 on 26-1-4.
//
#include <c++/13.3.1/bits/std_abs.h>

#include "message_center.h"
#include "robot.h"
#include "robot_def.h"
#include "boardCommunicate.h"
#include "buzzer.h"
#include "Communication.h"
#include "controller.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "remote_control.h"
#include "shoot.h"
#include "buzzer_alarm/buzzer_alarm.h"
#define AUTO_SHOOT_DELAY_MS 5000  // 延迟500ms再进入下一次发射
#define SLIDE_LENGTH 25.466f

static Publisher_t *auto_shoot_cmd_pub;            // 发射控制消息发布者
static Subscriber_t *shoot_cmd_sub;           // 发射控制消息订阅者
static Shoot_Ctrl_Cmd_s auto_shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;      // 接收来自发射的控制信息

static Publisher_t *auto_gimbal_cmd_pub;         // 云台控制消息发布者
static Gimbal_Ctrl_Cmd_s auto_gimbal_cmd_send;      // 传递给云台的控制信息

static PIDInstance autoaim_pid __attribute__((section(".dtcm_vars"))); //自动瞄准pid实例
static PIDInstance advshooter_pid __attribute__((section(".dtcm_vars"))); //自动瞄准pid实例
static float aim_output;
static float advshooter_output;

static global_struct *global_map;
static SlaveD_Rec_info_t *slaveRecInfo;  //接收上位机数据
static momoka_Rec_info_t *momokaRec_map; //接收镖板数据
static momoka_Send_info_t *momokaSend_map;
static DMMotorInstance *Stella_map;
static DJIMotorInstance *Liyuu_map;

static float auto_shoot_delay_time = 0;  // 延迟时间计数器

static uint8_t testtt=0;
uint8_t dart_order = 1;    //镖镖序号
static float ori_angle ; //自动发射开始时位置
static float std_energy_pos;

BuzzzerInstance *Dart_buzzer;
const Note_t Auto_Off_beep[] = {
    {NOTE_D5, 200},
    {NOTE_D5, 200},
    {NOTE_D5, 50},
    {NOTE_D5, 200},
{NOTE_D5, 100},
{NOTE_C5, 200},
{NOTE_A4, 100},
{NOTE_G4, 400},

};

Shoot_Ctrl_Cmd_s *get_auto_shoot_send() {
    return &auto_shoot_cmd_send;
}
void AutoAim_Init() {
    global_map = get_global();
    slaveRecInfo = get_slaveRec();
    momokaRec_map = get_momokaRec();
    momokaSend_map = get_momokaSend();

    memset(&autoaim_pid , 0 , sizeof(PIDInstance));
    memset(&advshooter_pid , 0 , sizeof(PIDInstance));

    Stella_map = get_Stella();
    Liyuu_map = get_Liyuu();
    BuzzerInit();
    Buzzer_config_s config = {
        .alarm_level = ALARM_LEVEL_HIGH, // 默认优先级
        .loudness = 1.0f,                   // 初始响度50%
        .octave = OCTAVE_4,                  // 兼容旧代码，可忽略
    };
    Dart_buzzer = BuzzerRegister(&config);
    // BuzzerSetLoudness(Dart_buzzer, 1.0f);

    PID_Init_Config_s aim_config={
        .Kp = 50.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 2500.0f,
    };
    PIDInit(&autoaim_pid, &aim_config);

    PID_Init_Config_s shooter_config={
        .Kp = 20.0f,
        .Ki = 0.02f,
        .Kd = 0.8f,
        .MaxOut = 700.0f,
    };
    PIDInit(&advshooter_pid, &shooter_config);

    auto_shoot_cmd_pub = PubRegister("auto_shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_cmd_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

    auto_gimbal_cmd_pub = PubRegister("auto_gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

void AutoAim_Task() {
    SubGetMessage(shoot_cmd_sub, &shoot_cmd_recv);
    switch (dart_order) {
        case 1:
            global_map->adv_force_set = 1415 + global_map->adv_CMD_data/800;
            global_map->Pixel_offset = 0;
            break;
        case 2:
            global_map->adv_force_set = 160 + global_map->adv_CMD_data/800;
            global_map->Pixel_offset = 0;
            break;
        case 3:
            global_map->adv_force_set = 165 + global_map->adv_CMD_data/800;
            global_map->Pixel_offset = 0;
            break;
        case 4:
            global_map->adv_force_set = 166 + global_map->adv_CMD_data/800;
            global_map->Pixel_offset = 0;
            break;
        default:
            break;
    }

    switch (shoot_cmd_recv.auto_mode) {
        case AUTO_OFF:
            auto_shoot_cmd_send.Liyuu_speed = 0.0f;
            auto_shoot_cmd_send.Stella_speed = 0.0f;
            auto_shoot_cmd_send.Trigger_Location = 0.0f;
            auto_gimbal_cmd_send.yaw_speed = 0.0f;
            auto_shoot_delay_time=0;
            momokaSend_map->ready_flag = 0;
            momokaSend_map->OnLine = Auto_Off;
            global_map->auto_debug_mode = Debug_on;

            if (global_map->std_energy_pos_flag) {
                std_energy_pos = (Liyuu_map->measure.total_angle)/50.0f;
                global_map->std_energy_pos_flag = 0;
            }
            break;
        case AUTO_ON:
            momokaSend_map->OnLine = AUTO_ON;
            BuzzerPlaySequence(Auto_Off_beep, Auto_Off_beep,
        sizeof(Auto_Off_beep)/sizeof(Auto_Off_beep[0]),
        ALARM_LEVEL_HIGH);

            aim_output = PIDCalculate(&autoaim_pid, (float)slaveRecInfo->Pixel_Error, 0.0f+(float)global_map->Pixel_offset);
            auto_gimbal_cmd_send.yaw_speed = 3.0f*aim_output;
            if (global_map->auto_shoot_wait_flag == 0 ){//|| abs(global_map->adv_force_set - momokaRec_map->Force_cur) > 2) {
                auto_shoot_cmd_send.Liyuu_speed = -advshooter_output*90.0f;
                // auto_shoot_cmd_send.Liyuu_speed = -(float)(global_map->adv_force_set - Sensor_force)*1440.0f;
            }
            switch (global_map->auto_debug_mode) {
                case Debug_on:
                    auto_shoot_cmd_send.Stella_speed = 1.0f;
                    advshooter_output = PIDCalculate(&advshooter_pid , Liyuu_map->measure.total_angle/50.0f , std_energy_pos);
                    if (momokaRec_map->slide_status == Slide_in_up) {
                        ori_angle = Stella_map->measure.total_angle;
                        auto_shoot_cmd_send.Stella_speed = 0.0f;
                        global_map->auto_debug_mode = Debug_off;
                    }
                    break;
                case Debug_off:
                    break;
            }
            if (global_map->auto_shoot_begin_flag == 1 ) {  //TODO：自动发射检测逻辑（半成品，后续待改进,使用状态机）
                if (global_map->auto_shoot_set_flag == 1 && global_map->auto_debug_mode == Debug_off && fabs(global_map->adv_force_set - momokaRec_map->Force_cur) < 2) {  //触发拨轮开始信号
                    global_map->auto_shoot_wait_flag = 1;
                    auto_shoot_cmd_send.Liyuu_speed = 0.0f;
                    global_map->auto_shoot_set_flag = 0;
                    auto_shoot_cmd_send.Stella_speed = -9.0f;
                }
                else if (Stella_map->measure.total_angle - ori_angle < -SLIDE_LENGTH) {
                    auto_shoot_cmd_send.Stella_speed = 11.0f;
                    global_map->auto_shoot_process_flag = 1;
                }
                else if (Stella_map->measure.total_angle - ori_angle > -0.2f && global_map->auto_shoot_process_flag == 1) {
                    auto_shoot_cmd_send.Stella_speed = 0.0f;
                    global_map->auto_shoot_process_flag = 0;
                    global_map->auto_shoot_ready_flag = 1;
                }
            }
            switch (global_map->auto_shoot_ready_flag) {
                case 0:
                    momokaSend_map->ready_flag = 0;
                    break;
                case 1:
                    momokaSend_map->ready_flag = 1;
                    // testtt++;
                    // if (global_map->fuck_them_all) {
                    auto_shoot_cmd_send.Trigger_Location=-80.0f;
                    if (auto_shoot_delay_time == 0) {
                        auto_shoot_delay_time = DWT_GetTimeline_ms();;  // 开始计时
                    }
                    // }
                    // 延迟足够后，自动进入下一次发射
                    // if (global_map->auto_shoot_end_flag){
                    if (DWT_GetTimeline_ms() - auto_shoot_delay_time > AUTO_SHOOT_DELAY_MS && auto_shoot_delay_time!=0) {
                        // testtt = 1;
                        auto_shoot_cmd_send.Trigger_Location=0.0f;
                        // global_map->fuck_them_all = 0;
                        global_map->auto_shoot_wait_flag = 0;
                        global_map->auto_shoot_ready_flag = 0;
                        global_map->auto_shoot_end_flag = 0;
                        global_map->auto_shoot_set_flag = 1;
                        auto_shoot_cmd_send.reload=!auto_shoot_cmd_send.reload;
                        auto_shoot_delay_time = 0;  // 重置计时器
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
    PubPushMessage(auto_gimbal_cmd_pub, (void *) &auto_gimbal_cmd_send);
    PubPushMessage(auto_shoot_cmd_pub, (void *) &auto_shoot_cmd_send);
}