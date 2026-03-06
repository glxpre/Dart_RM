// app
#include "robot_def.h"
#include "boardCommunicate.h"
#include "Communication.h"

#if defined(ONE_BOARD)||defined(GIMBAL_BOARD)
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "message_center.h"
#include "dji_motor.h"
#include "auto_routine.h"
#include "super_cap.h"

// bsp
#include "bsp_log.h"
#include "can_comm.h"
#include "dmmotor.h"
#include "referee_task.h"
// #include "referee_VT.h"
#include "rm_referee.h"

#include "robot.h"

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
#include "user_lib.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif

static RC_ctrl_t *rc_data;                // 遥控器数据,初始化时返回

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
// static Subscriber_t *auto_gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Ctrl_Cmd_s *auto_shoot_cmd_send_map;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

static referee_info_t *referee_data; // 用于获取裁判系统的数据

static global_struct *global_map;
// extern CANCommInstance *Cboard_can_comm;
//发送数据
static momoka_Send_info_t *momokaSendInfo;
//接收数据
static momoka_Rec_info_t *momokaRecInfo;

static uint8_t reload_flag = 1;    //换弹标志位，1为允许换弹
static uint8_t auto_flag = 1;    //自动发射介入标志位，1为介入
static uint8_t end_flag = 1;

static uint8_t testtt=0;

void RobotCMDInit() {
    global_map=get_global();
    auto_shoot_cmd_send_map = get_auto_shoot_send();
    // global_map->auto_shoot_end_flag = 1;
    momokaSendInfo=get_momokaSend();
    momokaSendInfo->cmdID=0;
    momokaSendInfo->dart_order_map=0;
    momokaSendInfo->OnLine=0;
    momokaSendInfo->ready_flag=0;
    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // vision_recv_data = VisionInit(); // 视觉通信串口
    //referee_VT_data = VTInit(&huart1);   // 图传通信串口
    referee_data = UITaskInit(&huart1, &ui_data); // 裁判系统初始化,会同时初始化UI
    //todo:这里的指针并非指定类型，有问题，还需要排查修改

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    // auto_gimbal_feed_sub = SubRegister("auto_gimbal_cmd", sizeof(Subscriber_t));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef GIMBAL_BOARD

#endif // GIMBAL_BOARD

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}


// static void update_ui_data() {
//     ui_data.chassis_mode = chassis_cmd_send.chassis_mode;
// }
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {
   // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下]，修改发射时拉力
    {
        reset_global();
        shoot_cmd_send.auto_mode = AUTO_OFF;
        momokaSendInfo->cmdID = Shoot_Force_change;
        // momokaSendInfo->CMD_data = rc_data[TEMP].rc.rocker_r1+5*rc_data[TEMP].rc.rocker_l1 ;  //左边杆高灵敏度，右边杆低灵敏度
        // CANCommSend(Cboard_can_comm , (uint8_t *)momokaSendInfo);
        // uartcommSendData((uint8_t *)&momokaSendInfo);
    }

    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中]，手动模式
    {
        reset_global();
        shoot_cmd_send.auto_mode = AUTO_OFF;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        // 云台参数,确定云台控制数据
        // 左侧开关状态为[中]
        if (switch_is_mid(rc_data[TEMP].rc.switch_left))
        {
            //
            shoot_cmd_send.Stella_speed = 1800.0f *19.0f* ((float) rc_data[TEMP].rc.rocker_l1 / 660);
            shoot_cmd_send.Liyuu_speed= 30000.0f * ((float) rc_data[TEMP].rc.rocker_r1 / 660);
            shoot_cmd_send.Trigger_Location=(-(float)rc_data[TEMP].rc.dial / 660)*80.0f;

        }
        // 左侧开关状态为[下]
        if (switch_is_down(rc_data[TEMP].rc.switch_left)) {
            gimbal_cmd_send.yaw_speed = -30000.0f * ((float) rc_data[TEMP].rc.rocker_l_/660);//todo：确认参数

            //换弹逻辑
            if ((-(float)rc_data[TEMP].rc.dial / 660)>0.2f&&reload_flag) {
                shoot_cmd_send.reload=!shoot_cmd_send.reload;
                reload_flag = 0;
            }
            else if ((-(float)rc_data[TEMP].rc.dial / 660)<0.2f)
                reload_flag = 1;
        }
        // uartcommSendData((uint8_t *)&momokaSendInfo);
    }//右侧开关为中

    else if (switch_is_up(rc_data[TEMP].rc.switch_right))  // 右侧开关状态[上],用于自动发射
    {
        momokaSendInfo->cmdID = Auto_Shoot;

        gimbal_cmd_send.gimbal_mode = GIMBAL_VISION_MODE;
        shoot_cmd_send.auto_mode = AUTO_ON;

        auto_shoot_cmd_send_map->liyuu_=Speed_Control;
        auto_shoot_cmd_send_map->Liyuu_speed= 30000.0f * ((float) rc_data[TEMP].rc.rocker_r1 / 660);  //调整丝杆

        // CANCommSend(Cboard_can_comm , (uint8_t *)momokaSendInfo);
        //向上拨拨轮，进入全自动模式
        if ((-(float)rc_data[TEMP].rc.dial / 660) > 0.5f && auto_flag) {
            global_map->auto_shoot_begin_flag = 1;
            global_map->auto_shoot_set_flag = 1;
            auto_flag = 0;
        }
        else if ((-(float)rc_data[TEMP].rc.dial / 660)<0.5f) {
            auto_flag = 1;
        }
        // // 下一次自动发射标志
        // if ((float) rc_data[TEMP].rc.rocker_l1 / 660 > 0.8 && end_flag) {
        //      global_map->auto_shoot_end_flag = 1;
        //      end_flag = 0;
        // }
        // else if ((float) rc_data[TEMP].rc.rocker_l1 / 660 < 0.8) {
        //     end_flag = 1;
        // }

        //检测拉力值是否进入范围

        // uartcommSendData((uint8_t *)&momokaSendInfo);
    }// 右侧开关状态[上]
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮向下打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler() {
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        shoot_cmd_send.stella_mode = Lost_Control;
        shoot_cmd_send.liyuu_=Liyuu_DISABLE;
        shoot_cmd_send.trigger_mode=disable;
        shoot_cmd_send.reload_mode=Reload_disable;

        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器左侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
        robot_state = ROBOT_READY;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;//自由速控模式
        shoot_cmd_send.stella_mode =Enable;
        shoot_cmd_send.trigger_mode=allow_fire,
        //shoot_cmd_send.shoot_mode=fire;
        shoot_cmd_send.liyuu_=Speed_Control;
        shoot_cmd_send.reload_mode = Reload_enable;

        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
    // 从其他应用获取回传数据
#ifdef GIMBAL_BOARD
    //chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    //SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    //SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    //update_ui_data();
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
    if (robot_state != ROBOT_STOP ){
        //todo:暂时为全遥控器控制，之后会加入自定义模式

        // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
        // if (switch_is_down(rc_data[TEMP].rc.switch_left) ||
        //     switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],[中],遥控器控制
        RemoteControlSet();
        // shoot_cmd_send.stella_mode =Enable;
        // else if (switch_is_up(rc_data[TEMP].rc.switch_left) &&
        //          switch_is_mid(rc_data[TEMP].rc.switch_right)) // 遥控器左侧开关状态为[上],键盘控制,遥控器右侧开关状态为[中],遥控器通道
        //     MouseKeySet();
        // else if (switch_is_up(rc_data[TEMP].rc.switch_left) &&
        //          switch_is_up(rc_data[TEMP].rc.switch_right)) // 遥控器左侧开关状态为[上],键盘控制，遥控器右侧开关状态为[上],图传链路通道
        //     VTRemoteControlSet();
    }

#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *) &shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *) &gimbal_cmd_send);

#endif

}
