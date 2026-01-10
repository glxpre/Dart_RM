//
// Created by 17946 on 26-1-4.
//
#include "message_center.h"
#include "robot.h"
#include "robot_def.h"
#include "boardCommunicate.h"
#include "Communication.h"

static Publisher_t *auto_shoot_cmd_pub;            // 发射控制消息发布者
static Shoot_Ctrl_Cmd_s auto_shoot_cmd_send;      // 传递给云台的控制信息
static Publisher_t *auto_gimbal_cmd_pub;            // 云台控制消息发布者
static Gimbal_Ctrl_Cmd_s auto_gimbal_cmd_send;      // 传递给云台的控制信息

static global_struct *global;
static SlaveD_Rec_info_t *slaveRecInfo;

void AutoAim_Init() {
    global = get_global();
    slaveRecInfo = get_slaveRec();
    auto_shoot_cmd_pub = PubRegister("auto_shoot_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    auto_gimbal_cmd_pub = PubRegister("auto_gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

void AutoAim_Task() {
    //视觉yaw瞄准
    if (slaveRecInfo->Pixel_Error >1 ) {
        auto_gimbal_cmd_send.yaw_speed = -3000.0f;
    }
    else if(slaveRecInfo->Pixel_Error< -1) {
        auto_gimbal_cmd_send.yaw_speed = 3000.0f;
    }
    else {
        auto_gimbal_cmd_send.yaw_speed = 0;
    }

    PubPushMessage(auto_gimbal_cmd_pub, (void *) &auto_gimbal_cmd_send);
}