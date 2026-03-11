//
// Created by 17946 on 26-1-4.
//
#include "message_center.h"
#include "robot.h"
#include "robot_def.h"
#include "boardCommunicate.h"
#include "Communication.h"
#include "controller.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "remote_control.h"
#include "shoot.h"
#define AUTO_SHOOT_DELAY_MS 5000  // 延迟500ms再进入下一次发射

static Publisher_t *auto_shoot_cmd_pub;            // 发射控制消息发布者
static Subscriber_t *shoot_cmd_sub;           // 发射控制消息订阅者
static Shoot_Ctrl_Cmd_s auto_shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;      // 接收来自发射的控制信息

static Publisher_t *auto_gimbal_cmd_pub;            // 云台控制消息发布者
static Gimbal_Ctrl_Cmd_s auto_gimbal_cmd_send;      // 传递给云台的控制信息

static PIDInstance autoaim_pid __attribute__((section(".dtcm_vars"))); //自动瞄准pid实例
float aim_output;

static global_struct *global_map;
static SlaveD_Rec_info_t *slaveRecInfo;  //接收上位机数据
static momoka_Rec_info_t *momokaRec_map; //接收镖板数据
static momoka_Send_info_t *momokaSend_map;
static DJIMotorInstance *Stella_map;

static float auto_shoot_delay_time = 0;  // 延迟时间计数器

static uint8_t testtt=0;
uint8_t dart_order = 1;    //镖镖序号
float ori_angle ; //自动发射开始时位置

Shoot_Ctrl_Cmd_s *get_auto_shoot_send() {
    return &auto_shoot_cmd_send;
}
void AutoAim_Init() {
    global_map = get_global();
    slaveRecInfo = get_slaveRec();
    momokaRec_map = get_momokaRec();
    momokaSend_map = get_momokaSend();
    // autoaim_pid = malloc(sizeof(PIDInstance));
    memset(&autoaim_pid , 0 , sizeof(PIDInstance));

    Stella_map = get_Stella();

    PID_Init_Config_s config={
        .Kp = 50.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 2500.0f,
    };
    PIDInit(&autoaim_pid, &config);

    auto_shoot_cmd_pub = PubRegister("auto_shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_cmd_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

    auto_gimbal_cmd_pub = PubRegister("auto_gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

void AutoAim_Task() {
    SubGetMessage(shoot_cmd_sub, &shoot_cmd_recv);

    switch (shoot_cmd_recv.auto_mode) {
        case AUTO_OFF:
            auto_shoot_cmd_send.Liyuu_speed = 0.0f;
            auto_shoot_cmd_send.Stella_speed = 0.0f;
            auto_shoot_cmd_send.Trigger_Location = 0.0f;
            auto_gimbal_cmd_send.yaw_speed = 0.0f;
            auto_shoot_delay_time=0;
            momokaSend_map->ready_flag = 0;
            break;
        case AUTO_ON:
            aim_output = PIDCalculate(&autoaim_pid, (float)slaveRecInfo->Pixel_Error, 0);
            auto_gimbal_cmd_send.yaw_speed = 3.0f*aim_output;
            // auto_shoot_cmd_send.Trigger_Location=(-(float)rc_data[TEMP].rc.dial / 660)*80.0f;
            if (global_map->auto_shoot_begin_flag == 1) {  //TODO：自动发射检测逻辑（半成品，后续待改进,使用状态机）
                if (global_map->auto_shoot_set_flag == 1) {  //触发拨轮开始信号
                    do {
                        auto_shoot_cmd_send.Liyuu_speed = (float)(global_map->adv_force - momokaRec_map->Force_cur)/200*70000;
                    } while (global_map->adv_force - momokaRec_map->Force_cur < 2);

                    ori_angle = Stella_map->measure.total_angle;
                    global_map->auto_shoot_set_flag = 0;
                    auto_shoot_cmd_send.Stella_speed = -14.0f*1800.0f;
                }
                if (Stella_map->measure.total_angle - ori_angle < -90080.21) {
                    auto_shoot_cmd_send.Stella_speed = 18.0f*1800.0f;
                    global_map->auto_shoot_process_flag = 1;
                }
                if (Stella_map->measure.total_angle - ori_angle > 0 && global_map->auto_shoot_process_flag == 1) {
                    auto_shoot_cmd_send.Stella_speed = 0.0f;
                    global_map->auto_shoot_process_flag = 0;
                    global_map->auto_shoot_ready_flag = 1;
                }

                // if (momokaRec_map->Force_cur < 250 && global_map->auto_shoot_ready_flag) {
                //     DWT_Delay(0.5f);
                //     global_map->auto_shoot_end_flag = 1;
                // }
            }
            switch (global_map->auto_shoot_ready_flag) {
                case 0:
                    momokaSend_map->ready_flag = 0;
                    break;
                case 1:
                    momokaSend_map->ready_flag = 1;
                    if (global_map->fuck_them_all) {
                        auto_shoot_cmd_send.Trigger_Location=-80.0f;
                        if (auto_shoot_delay_time == 0) {
                            auto_shoot_delay_time = DWT_GetTimeline_ms();;  // 开始计时
                        }
                    }
                    // 延迟足够后，自动进入下一次发射
                    // if (global_map->auto_shoot_end_flag){
                    if (DWT_GetTimeline_ms() - auto_shoot_delay_time > AUTO_SHOOT_DELAY_MS && auto_shoot_delay_time!=0) {
                        // testtt = 1;
                        auto_shoot_cmd_send.Trigger_Location=0.0f;
                        // global_map->fuck_them_all = 0;
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