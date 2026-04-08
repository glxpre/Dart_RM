//
// Created by 17946 on 26-1-4.
//
#include "auto_routine.h"

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
#define AUTO_SHOOT_DELAY_MS 1000  // 延迟500ms再进入下一次发射
#define SLIDE_LENGTH 25.400f
#define MAX_STATE 6
#define LIMIT_ONE(x)  (( (x) > 1 ) ? 0 : (x))
#define LIMIT_MAX(x)  (( (x) > (MAX_STATE-1) ) ? 0 : (x))

static Publisher_t *auto_shoot_cmd_pub;            // 发射控制消息发布者
static Subscriber_t *shoot_cmd_sub;           // 发射控制消息订阅者
static Shoot_Ctrl_Cmd_s auto_shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;      // 接收来自发射的控制信息

static Publisher_t *auto_gimbal_cmd_pub;         // 云台控制消息发布者
static Gimbal_Ctrl_Cmd_s auto_gimbal_cmd_send;      // 传递给云台的控制信息

static PIDInstance autoaim_pid __attribute__((section(".dtcm_vars"))); //自动瞄准pid实例
static PIDInstance force_shooter_pid __attribute__((section(".dtcm_vars"))); //自动瞄准pid实例
static PIDInstance pos_shooter_pid __attribute__((section(".dtcm_vars"))); //自动瞄准pid实例
static float aim_output;
static float force_shooter_output;
static float pos_shooter_output;

static global_struct *global_map;
static SlaveD_Rec_info_t *slaveRecInfo;  //接收上位机数据
static momoka_Rec_info_t *momokaRec_map; //接收镖板数据
static momoka_Send_info_t *momokaSend_map;
static DMMotorInstance *Stella_map;
static DJIMotorInstance *Liyuu_map;

static float auto_shoot_delay_time = 0;  // 延迟时间计数器

static int testtt=0;
uint8_t dart_order = 1;    //镖镖序号
extern int Sensor_force;
static float ori_angle ; //自动发射开始时位置
static float std_energy_pos;  //储能丝杆标准位
static float energy_angle_map;  //倍数缩放的丝杆角度

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
    memset(&force_shooter_pid , 0 , sizeof(PIDInstance));
    memset(&pos_shooter_pid , 0 , sizeof(PIDInstance));

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
        .Kp = 100.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 3000.0f,
    };
    PIDInit(&autoaim_pid, &aim_config);

    PID_Init_Config_s force_shooter_config={
        .Kp = 35.0f,
        .Ki = 0.0f,
        .Kd = 0.8f,
        .MaxOut = 850.0f,
    };
    PIDInit(&force_shooter_pid, &force_shooter_config);

    PID_Init_Config_s pos_shooter_config={
        .Kp = 20.0f,
        .Ki = 0.0f,
        .Kd = 0.8f,
        .MaxOut = 5000.0f,
    };
    PIDInit(&pos_shooter_pid, &pos_shooter_config);

    auto_shoot_cmd_pub = PubRegister("auto_shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_cmd_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

    auto_gimbal_cmd_pub = PubRegister("auto_gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}


static int current_state_MAIN = 0;
static int current_state_AUTO_ON=DEBUG_STATE_INDEX;
static int dart_number=4;

void DEBUG_STATE(void) {
    if (dart_number>0) {
        auto_shoot_cmd_send.Stella_speed = 1.0f;
        auto_shoot_cmd_send.Liyuu_speed = pos_shooter_output*12.0f;
        if (momokaRec_map->slide_status == Slide_in_up ){//&& fabs(auto_shoot_cmd_send.Liyuu_speed - 0.0f) < 0.5f) {
            ori_angle = Stella_map->measure.total_angle;
            auto_shoot_cmd_send.Stella_speed = 0.0f;
        }
        if (momokaRec_map->slide_status == Slide_in_up && fabsf(pos_shooter_output - 0.0f) < 30.0f&&global_map->auto_shoot_begin_flag==1) {
            auto_shoot_cmd_send.Liyuu_speed = 0.0f;
            current_state_AUTO_ON=SLIP_DOWN_STATE_INDEX;
        }
    }
}
void SLIP_DOWN_STATE(void) {
    auto_shoot_cmd_send.Stella_speed = -12.0f;
    if (Stella_map->measure.total_angle - ori_angle < -SLIDE_LENGTH) {
        current_state_AUTO_ON=SLIP_UP_STATE_INDEX;
    }
}
void SLIP_UP_STATE(void) {
    auto_shoot_cmd_send.Stella_speed = 14.0f;
    if (Stella_map->measure.total_angle - ori_angle > -0.25f ) {
        current_state_AUTO_ON=SCREW_ADJUST_FORCE_STATE_INDEX;
        auto_shoot_cmd_send.Stella_speed = 0.0f;
    }
}
void SCREW_ADJUST_FORCE_STATE(void) {
    auto_shoot_cmd_send.Stella_speed = 0.0f;
    force_shooter_output = PIDCalculate(&force_shooter_pid, (float)Sensor_force, (float)global_map->adv_force_set);
    auto_shoot_cmd_send.Liyuu_speed = -force_shooter_output*21.5f;
    if (abs(testtt-0) < 2) {
        auto_shoot_cmd_send.Liyuu_speed =0;
        current_state_AUTO_ON=TRIGGER_ON_STATE_INDEX;
    }
}
void TRIGGER_ON_STATE(void) {
    auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    auto_shoot_cmd_send.Trigger_Location=-80.0f;
    current_state_AUTO_ON=RELOAD_STATE_INDEX;
}
static int reload_start_flag=1;
void RELOAD_STATE(void) {
    if (reload_start_flag) {
        auto_shoot_delay_time = DWT_GetTimeline_ms();
        reload_start_flag=0;
    }  // 开始计时
    if (DWT_GetTimeline_ms() - auto_shoot_delay_time > AUTO_SHOOT_DELAY_MS && auto_shoot_delay_time!=0) {
        // testtt = 1;
        auto_shoot_cmd_send.Trigger_Location=0.0f;
        auto_shoot_cmd_send.reload=!auto_shoot_cmd_send.reload;
        auto_shoot_delay_time = 0;  // 重置计时器
        dart_number-=1;
        reload_start_flag=1;
        current_state_AUTO_ON=DEBUG_STATE_INDEX;
    }
}
static void (*auto_aim_state_table_AUTO_ON[])(void)={
    DEBUG_STATE,
    SLIP_DOWN_STATE,
    SLIP_UP_STATE,
    SCREW_ADJUST_FORCE_STATE,
    TRIGGER_ON_STATE,
    RELOAD_STATE
};
void AUTO_OFF_FUNCTION(void) {
    auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    auto_shoot_cmd_send.Stella_speed = 0.0f;
    auto_shoot_cmd_send.Trigger_Location = 0.0f;
    auto_gimbal_cmd_send.yaw_speed = 0.0f;
    auto_shoot_delay_time=0;
    momokaSend_map->ready_flag = 0;
    momokaSend_map->OnLine = Auto_Off;
    global_map->auto_debug_mode = Debug_on;
    dart_number=4;
    if (global_map->std_energy_pos_flag) {
        std_energy_pos = Liyuu_map->measure.total_angle/50.0f;
        global_map->std_energy_pos_flag = 0;
    }
    if (shoot_cmd_recv.auto_mode==AUTO_ON) {
        current_state_MAIN=1;
    }
}
void AUTO_ON_FUNCTION(void) {
    momokaSend_map->OnLine = AUTO_ON;
    BuzzerPlaySequence(Auto_Off_beep, Auto_Off_beep,
sizeof(Auto_Off_beep)/sizeof(Auto_Off_beep[0]),
ALARM_LEVEL_HIGH);
    //yaw轴对准
    aim_output = PIDCalculate(&autoaim_pid, (float)slaveRecInfo->Pixel_Error, 0.0f+(float)global_map->Pixel_offset);
    auto_gimbal_cmd_send.yaw_speed = 3.0f*aim_output;
    pos_shooter_output = PIDCalculate(&pos_shooter_pid , Liyuu_map->measure.total_angle/50.0f , std_energy_pos);
    force_shooter_output = PIDCalculate(&force_shooter_pid, (float)Sensor_force, (float)global_map->adv_force_set);
    // if (global_map->auto_shoot_wait_flag == 0 ){//|| abs(global_map->adv_force_set - momokaRec_map->Force_cur) > 2) {
    //     auto_shoot_cmd_send.Liyuu_speed = -pos_shooter_output*90.0f;
    //     // auto_shoot_cmd_send.Liyuu_speed = -(float)(global_map->adv_force_set - Sensor_force)*1440.0f;
    // }

    if (auto_aim_state_table_AUTO_ON[current_state_AUTO_ON] != NULL) {
        auto_aim_state_table_AUTO_ON[current_state_AUTO_ON]();
    }
}

static void (*auto_aim_state_table_main[])(void)={AUTO_OFF_FUNCTION,AUTO_ON_FUNCTION};

void AutoAim_Task() {
    SubGetMessage(shoot_cmd_sub, &shoot_cmd_recv);
    testtt = Sensor_force - global_map->adv_force_set;
    switch (dart_order) {
        case 1:
            global_map->adv_force_set = 1348+3+4+ global_map->adv_CMD_data/10000;
            global_map->Pixel_offset = -15-16+global_map->CMD_Pixel_offset/10000;
            break;
        case 2:
            global_map->adv_force_set = 1337+21+4+ global_map->adv_CMD_data/10000;
            global_map->Pixel_offset = -15-1-3+global_map->CMD_Pixel_offset/10000;
            break;
        case 3:
            global_map->adv_force_set = 1344+18+5 +global_map->adv_CMD_data/10000;
            global_map->Pixel_offset = 6+1-8+global_map->CMD_Pixel_offset/10000;
            break;
        case 4:
            global_map->adv_force_set = 1336+7+9+ global_map->adv_CMD_data/10000;
            global_map->Pixel_offset = -9-1-12+global_map->CMD_Pixel_offset/10000;
            break;
        default:
            break;
    }

    if (shoot_cmd_recv.auto_mode != AUTO_ON) {
        current_state_MAIN = 0;
    }

    if (auto_aim_state_table_main[LIMIT_ONE(current_state_MAIN)] != NULL) {
        auto_aim_state_table_main[LIMIT_ONE(current_state_MAIN)]();
    }


    // switch (shoot_cmd_recv.auto_mode) {
    //     case AUTO_OFF:
    //         auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    //         auto_shoot_cmd_send.Stella_speed = 0.0f;
    //         auto_shoot_cmd_send.Trigger_Location = 0.0f;
    //         auto_gimbal_cmd_send.yaw_speed = 0.0f;
    //         auto_shoot_delay_time=0;
    //         momokaSend_map->ready_flag = 0;
    //         momokaSend_map->OnLine = Auto_Off;
    //         global_map->auto_debug_mode = Debug_on;
    //
    //         if (global_map->std_energy_pos_flag) {
    //             std_energy_pos = Liyuu_map->measure.total_angle/50.0f;
    //             global_map->std_energy_pos_flag = 0;
    //         }
    //         break;
    //     case AUTO_ON:
    //         momokaSend_map->OnLine = AUTO_ON;
    //         BuzzerPlaySequence(Auto_Off_beep, Auto_Off_beep,
    //     sizeof(Auto_Off_beep)/sizeof(Auto_Off_beep[0]),
    //     ALARM_LEVEL_HIGH);
    //         //yaw轴对准
    //         aim_output = PIDCalculate(&autoaim_pid, (float)slaveRecInfo->Pixel_Error, 0.0f+(float)global_map->Pixel_offset);
    //         auto_gimbal_cmd_send.yaw_speed = 3.0f*aim_output;
    //         pos_shooter_output = PIDCalculate(&pos_shooter_pid , Liyuu_map->measure.total_angle/50.0f , std_energy_pos);
    //         force_shooter_output = PIDCalculate(&force_shooter_pid, (float)Sensor_force, (float)global_map->adv_force_set);
    //         // if (global_map->auto_shoot_wait_flag == 0 ){//|| abs(global_map->adv_force_set - momokaRec_map->Force_cur) > 2) {
    //         //     auto_shoot_cmd_send.Liyuu_speed = -pos_shooter_output*90.0f;
    //         //     // auto_shoot_cmd_send.Liyuu_speed = -(float)(global_map->adv_force_set - Sensor_force)*1440.0f;
    //         // }
    //         switch (global_map->auto_debug_mode) {
    //             case Debug_on:
    //                 auto_shoot_cmd_send.Stella_speed = 1.0f;
    //                 auto_shoot_cmd_send.Liyuu_speed = pos_shooter_output*12.0f;
    //                 if (momokaRec_map->slide_status == Slide_in_up ){//&& fabs(auto_shoot_cmd_send.Liyuu_speed - 0.0f) < 0.5f) {
    //                     ori_angle = Stella_map->measure.total_angle;
    //                     auto_shoot_cmd_send.Stella_speed = 0.0f;
    //                 }
    //                 if (momokaRec_map->slide_status == Slide_in_up && fabsf(pos_shooter_output - 0.0f) < 30.0f) {
    //                     auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    //                     global_map->auto_debug_mode = Debug_ready;
    //                 }
    //                 break;
    //             case Debug_ready:
    //                 break;
    //         }
    //         if (global_map->auto_shoot_begin_flag == 1 ) {  //TODO：自动发射检测逻辑（半成品，后续待改进,使用状态机）
    //             if (global_map->auto_shoot_set_flag == 1 && global_map->auto_debug_mode == Debug_ready ){//&& fabs(global_map->adv_force_set - momokaRec_map->Force_cur) < 2) {  //触发拨轮开始信号
    //                 // global_map->auto_shoot_wait_flag = 1;
    //                 // auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    //                 global_map->auto_shoot_set_flag = 0;
    //                 auto_shoot_cmd_send.Stella_speed = -9.0f;
    //             }
    //             else if (Stella_map->measure.total_angle - ori_angle < -SLIDE_LENGTH) {
    //                 auto_shoot_cmd_send.Stella_speed = 11.0f;
    //                 auto_shoot_cmd_send.Liyuu_speed = force_shooter_output*15.0f;
    //                 global_map->auto_shoot_process_flag = 1;
    //             }
    //             else if (Stella_map->measure.total_angle - ori_angle > -0.25f && global_map->auto_shoot_process_flag == 1) {
    //                 auto_shoot_cmd_send.Stella_speed = 0.0f;
    //                 global_map->auto_shoot_process_flag = 0;
    //                 global_map->auto_shoot_ready_flag = 1;
    //             }
    //         }
    //         switch (global_map->auto_shoot_ready_flag) {
    //             case 0:
    //                 momokaSend_map->ready_flag = 0;
    //                 if (global_map->auto_shoot_set_flag == 0 && global_map->auto_routine_end_flag == 1) {
    //                     auto_shoot_cmd_send.Liyuu_speed = pos_shooter_output*18.0f;
    //                 }
    //                 if (fabsf(pos_shooter_output - 0.0f) < 30.0f && global_map->auto_shoot_wait_flag == 1) {
    //                     global_map->auto_routine_end_flag = 0;
    //                     global_map->auto_shoot_set_flag = 1;
    //                     global_map->auto_shoot_wait_flag = 0;
    //                 }
    //                 break;
    //             case 1:
    //                 momokaSend_map->ready_flag = 1;
    //                 // testtt++;
    //                 // if (global_map->fuck_them_all) {
    //                 if (abs(testtt - 0) < 2) {
    //                     auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    //                     auto_shoot_cmd_send.Trigger_Location=-80.0f;
    //                     global_map->auto_shoot_end_flag = 1;
    //                 }
    //                 else if (abs(testtt - 0) > 2 && global_map->auto_shoot_end_flag == 0){
    //                     auto_shoot_cmd_send.Liyuu_speed = -force_shooter_output*15.0f;
    //                 }
    //                 if (auto_shoot_delay_time == 0 && global_map->auto_shoot_end_flag == 1) {
    //                     auto_shoot_delay_time = DWT_GetTimeline_ms();;  // 开始计时
    //                 }
    //                 // }
    //                 // 延迟足够后，自动进入下一次发射
    //                 // if (global_map->auto_shoot_end_flag){
    //                 if (DWT_GetTimeline_ms() - auto_shoot_delay_time > AUTO_SHOOT_DELAY_MS && auto_shoot_delay_time!=0) {
    //                     // testtt = 1;
    //                     auto_shoot_cmd_send.Trigger_Location=0.0f;
    //                     global_map->auto_routine_end_flag = 1;
    //                     global_map->auto_shoot_ready_flag = 0;
    //                     global_map->auto_shoot_end_flag = 0;
    //                     global_map->auto_shoot_wait_flag = 1;
    //                     auto_shoot_cmd_send.reload=!auto_shoot_cmd_send.reload;
    //                     auto_shoot_delay_time = 0;  // 重置计时器
    //                 }
    //                 break;
    //             default:
    //                 break;
    //         }
    //         break;
    //     default:
    //         break;
    // }
    PubPushMessage(auto_gimbal_cmd_pub, (void *) &auto_gimbal_cmd_send);
    PubPushMessage(auto_shoot_cmd_pub, (void *) &auto_shoot_cmd_send);
}