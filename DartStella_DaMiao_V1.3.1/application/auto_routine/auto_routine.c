//
// Created by 17946 on 26-1-4.
//
#include "message_center.h"
#include "robot.h"
#include "robot_def.h"
#include "boardCommunicate.h"
#include "Communication.h"
#include "controller.h"
#include "dmmotor.h"
#include "shoot.h"

static Publisher_t *auto_shoot_cmd_pub;            // 发射控制消息发布者
static Subscriber_t *auto_shoot_cmd_sub2;           // 发射控制消息订阅者
static Shoot_Ctrl_Cmd_s auto_shoot_cmd_send;      // 传递给云台的控制信息
static Shoot_Ctrl_Cmd_s auto_shoot_cmd_recv;      // 接收来自云台的控制信息

static Publisher_t *auto_gimbal_cmd_pub;            // 云台控制消息发布者
static Gimbal_Ctrl_Cmd_s auto_gimbal_cmd_send;      // 传递给云台的控制信息

static PIDInstance autoaim_pid __attribute__((section("dtcm_vars"))); //自动瞄准pid实例
float aim_output;

static global_struct *global_map;
static SlaveD_Rec_info_t *slaveRecInfo;  //接收上位机数据
static DMMotorInstance *Stella_map;

uint8_t dart_order = 1;    //镖镖序号
float ori_angle ; //自动发射开始时位置

void AutoAim_Init() {
    global_map = get_global();
    slaveRecInfo = get_slaveRec();
    // autoaim_pid = malloc(sizeof(PIDInstance));
    memset(&autoaim_pid , 0 , sizeof(PIDInstance));

    Stella_map = get_Stella();

    PID_Init_Config_s config={
        .Kp = 5.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 5000.0f,
    };
    PIDInit(&autoaim_pid, &config);

    auto_shoot_cmd_pub = PubRegister("auto_shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    auto_shoot_cmd_sub2 = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

    auto_gimbal_cmd_pub = PubRegister("auto_gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

void AutoAim_Task() {
    SubGetMessage(auto_shoot_cmd_sub2, &auto_shoot_cmd_recv);

    switch (auto_shoot_cmd_recv.auto_mode) {
        case AUTO_OFF:
            auto_shoot_cmd_send.Liyuu_speed = 0.0f;
            auto_shoot_cmd_send.Stella_speed = 0.0f;
            auto_shoot_cmd_send.Trigger_Location = 0.0f;
            break;
        case AUTO_ON:
            aim_output = PIDCalculate(&autoaim_pid, (float)slaveRecInfo->Pixel_Error, 0);
            auto_gimbal_cmd_send.yaw_speed = 30.0f*aim_output;
            if (global_map->auto_shoot_begin_flag == 1) {  //TODO：自动发射检测逻辑（半成品，后续待改进,使用状态机）
                if (global_map->auto_shoot_set_flag == 1) {  //触发拨轮开始信号
                    ori_angle = Stella_map->measure.total_angle;
                    global_map->auto_shoot_set_flag = 0;
                    auto_shoot_cmd_send.Stella_speed = 12.0f;
                }
                if (Stella_map->measure.total_angle - ori_angle > 27.00) {
                    auto_shoot_cmd_send.Stella_speed = -17.0f;
                    global_map->auto_shoot_process_flag = 1;
                }
                if (Stella_map->measure.total_angle - ori_angle < 0 && global_map->auto_shoot_process_flag == 1) {
                    auto_shoot_cmd_send.Stella_speed = 0.0f;
                    global_map->auto_shoot_process_flag = 0;
                    global_map->auto_shoot_ready_flag = 1;
                }
            }
            break;
        default:
            break;
    }
    PubPushMessage(auto_gimbal_cmd_pub, (void *) &auto_gimbal_cmd_send);
    PubPushMessage(auto_shoot_cmd_pub, (void *) &auto_shoot_cmd_send);
}