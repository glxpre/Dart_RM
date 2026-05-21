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
#include "rm_referee.h"
#define AUTO_SHOOT_DELAY_MS 1000  // 延迟500ms再进入下一次发射
#define AUTO_SHOOT_WAIT_MS 100
volatile static float SLIDE_LENGTH=25.500f;
#define MAX_STATE 7
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
static float tick_delay_time = 0;  // 延迟时间计数器

static float testtt=0;
uint8_t dart_order = 1;    //镖镖序号
extern float Sensor_force;
static float ori_angle ; //自动发射开始时位置
static float std_energy_pos;  //储能丝杆标准位
static float energy_angle_map;  //倍数缩放的丝杆角度

static int current_state_MAIN = 0;
static int current_state_AUTO_ON=DEBUG_STATE_INDEX;
static int dart_number=4;
static int reload_start_flag=1;
static int shoot_wait_start_flag=1;
static int tick_start_flag=1;
static int cd_shoot=0;

static int batch_darts_fired=0;
static int batch_size=3;
// static volatile int dart_door_flag = 0;
// static int dart_door_open_flag=0;//todo:把标志位搞个结构体，分个类
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
        .Kp = 120.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 3500.0f,
    };
    PIDInit(&autoaim_pid, &aim_config);

    PID_Init_Config_s force_shooter_config={
        .Kp = 35.0f,
        .Ki = 0.0f,
        .Kd = 0.8f,
        .MaxOut = 800.0f,
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

static int auto_shoot_cmd_flag=0;




typedef enum {
    Dart_station_close,
    Dart_station_open,
    Dart_station_active_close2open,
    Dart_station_active_open2close,
}Dart_station_state_enum;
void check_state(Dart_station_state_enum* state,Dart_station_state_enum* last_state) {
    switch (referee_info.DartControlData.dart_station_state) {
        case(1) :
            if (*state!=Dart_station_close) {
                *last_state=*state;
            }
            *state=Dart_station_close;
            break;
        case(0) :
            if (*state!=Dart_station_open) {
                *last_state=*state;
            }
            *state=Dart_station_open;
            break;
        case(2) :
            if (*state!=Dart_station_active_close2open||*state!=Dart_station_active_open2close) {
                *last_state=*state;
            }
            if (*state==Dart_station_open) {
                *state=Dart_station_active_open2close;
            }
            if (*state==Dart_station_close) {
                *state=Dart_station_active_close2open;
            }
            break;
    }
}
struct {
    Dart_station_state_enum state;
    Dart_station_state_enum last_state;
    void (*check_state)(Dart_station_state_enum* state,Dart_station_state_enum* last_state);
}dart_station;

//Todo:把这个时间延迟的逻辑封装一下，写了一点，懒得写了

// void tick_time_start(float* tick_delay_time,int* tick_start_flag,float* tick_start_time){
//     *tick_start_time= DWT_GetTimeline_ms();
// }
// float tick_delay_time(float* tick_delay_time,int* tick_start_flag,float* tick_start_time) {
//     tick_start_time(tick_delay_time,tick_start_flag,tick_start_time);
// }
// struct struct_time_tick{
//     float tick_delay_time;
//     int tick_start_flag;
//     float tick_start_time;
//     float (*tick_delay_time)(float* tick_delay_time,int* tick_start_flag,float* tick_start_time);
// };
//
// struct struct_time_tick time_tick={
//     .tick_delay_time =0,
//     .tick_start_flag = 1,
// };


int yaw_abnormol(){
    if (slaveRecInfo->Pixel_Error==-660)return 1;
    if (slaveRecInfo->Pixel_Error==0&&slaveRecInfo->Distance==0)return 1;
    return 0;
}
//todo:用freertos重写整个代码，使其真正实时性
void DEBUG_STATE(void) {
        if (dart_number>0) {
            auto_shoot_cmd_send.Stella_speed = 1.0f;
            auto_shoot_cmd_send.Liyuu_speed = pos_shooter_output*12.0f;
            if (global_map->auto_shoot_begin_flag==1)auto_shoot_cmd_flag=1;
        //     if (referee_info.DartControlData.dart_station_state==2 && referee_info.GameState.game_progress == 4) {
        //         dart_door_flag=1;
        //         if (tick_start_flag) {
        //             tick_delay_time = DWT_GetTimeline_ms();//裁判
        //             tick_start_flag=0;
        //         }
        //     }
        // if (DWT_GetTimeline_ms() - tick_delay_time > 3000&& tick_delay_time!=0) {
        //     if (referee_info.DartControlData.dart_station_state==0 &&dart_door_flag&& referee_info.GameState.game_progress == 4) {
        //         auto_shoot_cmd_flag=1;
        //         dart_door_flag = 0;
        //         tick_delay_time=0;
        //         tick_start_flag=1;
        //     }
        //     if (DWT_GetTimeline_ms() - tick_delay_time > 6000&& tick_delay_time!=0) {
        //         if (referee_info.DartControlData.dart_station_state!=0) {
        //             tick_start_flag = 1;
        //             tick_delay_time = 0;
        //             dart_door_flag = 0;
        //         }
        //     }
        // }
            if (referee_info.GameState.game_progress==4) {
                dart_station.check_state(&dart_station.state,&dart_station.last_state);
                if (dart_station.state==Dart_station_active_close2open&&dart_station.last_state==Dart_station_close) {
                    if (tick_start_flag) {
                        tick_delay_time = DWT_GetTimeline_ms();//裁判
                        tick_start_flag=0;
                    }
                }
                if (dart_station.state==Dart_station_open&&dart_station.last_state==Dart_station_active_close2open
                    &&(DWT_GetTimeline_ms()-tick_delay_time)>500) {
                    auto_shoot_cmd_flag=1;
                    tick_delay_time=0;
                    tick_start_flag=1;
                }
            }

            if (momokaRec_map->slide_status == Slide_in_up ){//&& fabs(auto_shoot_cmd_send.Liyuu_speed - 0.0f) < 0.5f) {
                ori_angle = Stella_map->measure.total_angle;
                auto_shoot_cmd_send.Stella_speed = 0.0f;
            }
            if (momokaRec_map->slide_status == Slide_in_up &&  auto_shoot_cmd_flag  &&fabsf(pos_shooter_output - 0.0f) < 30.0f
                &&abs(global_map->Pixel_offset-slaveRecInfo->Pixel_Error)<1.0&&referee_info.DartControlData.dart_station_state==0&& referee_info.GameState.game_progress == 4) {
                auto_shoot_cmd_send.Liyuu_speed = 0.0f;
                momokaSend_map->magnet_flag=1;
                current_state_AUTO_ON=SLIP_DOWN_STATE_INDEX;
            }
        }
    }

void SLIP_DOWN_STATE(void) {
    if (tick_start_flag) {
        tick_delay_time = DWT_GetTimeline_ms();
        tick_start_flag=0;
    }
    if (DWT_GetTimeline_ms() - tick_delay_time > 15000&& tick_delay_time!=0) {
        tick_delay_time = 0;
    }
    auto_shoot_cmd_send.Stella_speed = -12.0f;
    if (Stella_map->measure.total_angle - ori_angle < -25.20f) {
        tick_delay_time=0;
        tick_start_flag=1;
        current_state_AUTO_ON=SLIP_UP_STATE_INDEX;
    }

}
void SLIP_UP_STATE(void) {
    auto_shoot_cmd_send.Stella_speed = 14.0f;
    if (tick_start_flag) {
        tick_delay_time = DWT_GetTimeline_ms();
        tick_start_flag=0;
    }
    if (DWT_GetTimeline_ms() - tick_delay_time > 15000&& tick_delay_time!=0) {
        tick_delay_time = 0;
    }
    if (Stella_map->measure.total_angle - ori_angle > -0.40f ) {
        tick_delay_time=0;
        tick_start_flag=1;
        current_state_AUTO_ON=SCREW_ADJUST_FORCE_STATE_INDEX;
        auto_shoot_cmd_send.Stella_speed = 0.0f;
    }
}
void SCREW_ADJUST_FORCE_STATE(void) {
    auto_shoot_cmd_send.Stella_speed = 0.0f;
    force_shooter_output = PIDCalculate(&force_shooter_pid, Sensor_force, global_map->adv_force_set);
    auto_shoot_cmd_send.Liyuu_speed = -force_shooter_output*17.0f;
    if (tick_start_flag) {
        tick_delay_time = DWT_GetTimeline_ms();
        tick_start_flag=0;
    }
    if (DWT_GetTimeline_ms() - auto_shoot_delay_time > 8000&& auto_shoot_delay_time!=0) {
        tick_delay_time = 0;
        current_state_AUTO_ON=EMERGENCY_STATE_INDEX;
    }
    if (Sensor_force>=1500||Sensor_force<=300) {
        current_state_AUTO_ON=EMERGENCY_STATE_INDEX;
    }
    if (abs(testtt) < 1.0f) {
        // auto_shoot_cmd_send.Liyuu_speed =0;
        // if (shoot_wait_start_flag) {
        //     auto_shoot_delay_time = DWT_GetTimeline_ms();
        //     shoot_wait_start_flag=0;
        // }
        // if (DWT_GetTimeline_ms() - auto_shoot_delay_time > AUTO_SHOOT_WAIT_MS && auto_shoot_delay_time!=0) {
        //     if (abs(testtt) < 1.0f) {
        //         shoot_wait_start_
        //         flag=1;
        //         auto_shoot_delay_time = 0;
        tick_start_flag=1;
        tick_delay_time=0;
        current_state_AUTO_ON=TRIGGER_ON_STATE_INDEX;
        //     }
        // }
    }
}
void TRIGGER_ON_STATE(void) {
    auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    auto_shoot_cmd_send.Trigger_Location=-80.0f;
    momokaSend_map->magnet_flag=0;
    current_state_AUTO_ON=RELOAD_STATE_INDEX;
}

void RELOAD_STATE(void) {
    if (reload_start_flag) {
        auto_shoot_delay_time = DWT_GetTimeline_ms();//todo:这里auto_shoot_delay_time与tick_delay_time重复了
        reload_start_flag=0;
    }  // 开始计时
    if (DWT_GetTimeline_ms() - auto_shoot_delay_time > AUTO_SHOOT_DELAY_MS && auto_shoot_delay_time!=0) {
        // testtt = 1;
        auto_shoot_cmd_send.Trigger_Location=0.0f;
        auto_shoot_cmd_send.reload=!auto_shoot_cmd_send.reload;
        auto_shoot_delay_time = 0;  // 重置计时器
        dart_number-=1;
        reload_start_flag=1;

        batch_darts_fired +=1;
        if (dart_number<=0)
        {
            current_state_AUTO_ON=DEBUG_STATE_INDEX;
        }else if (batch_darts_fired>=batch_size)
        {
            current_state_AUTO_ON=WAIT_TRIGGER_STATE_INDEX;
        }else
        {
            current_state_AUTO_ON=DEBUG_STATE_INDEX;
        }

    }
}
void WAIT_TRIGGER_STATE(void)
{
    auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    auto_shoot_cmd_send.Stella_speed = 0.0f;
    auto_shoot_cmd_send.Trigger_Location = 0.0f;
    if (referee_info.DartControlData.dart_station_state==2) {
        dart_door_flag=1;
        auto_gimbal_cmd_send.yaw_speed=0;
        if (tick_start_flag) {
            tick_delay_time = DWT_GetTimeline_ms();//裁判
            tick_start_flag=0;
        }
    }
    if (dart_door_flag) {
        //todo:把标志位规范一下，代码不同人写的，用了功能相同的两个标志位，简化为一个标志位
        if (referee_info.DartControlData.dart_station_state==0)
            dart_door_open_flag=1;
        if (referee_info.DartControlData.dart_station_state==1)
            dart_door_flag=0;
    }

    if (dart_door_open_flag) {
        if (slaveRecInfo->Pixel_Error==-660) {
            auto_gimbal_cmd_send.yaw_speed=-6000.0f;
            cd_shoot=1;
        }
        else{
            if (cd_shoot==1) {
                cd_shoot=2;
            }
            auto_gimbal_cmd_send.yaw_speed = 0.0f;
            if (global_map->auto_shoot_begin_flag == 1 || auto_shoot_cmd_flag) {
                batch_darts_fired = 0;
                global_map->auto_shoot_begin_flag = 0;
                auto_shoot_cmd_flag = 0;
                dart_door_open_flag=0;
             if (cd_shoot==2) {
                 std_energy_pos-=9500;
             }
                // SLIDE_LENGTH=25.450f;
                current_state_AUTO_ON = DEBUG_STATE_INDEX;
            }
        }
    }
}
void EMERGENCY_STATE(void) {
    auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    auto_shoot_cmd_send.Stella_speed = 0.0f;
}
static void (*auto_aim_state_table_AUTO_ON[])(void)={
    DEBUG_STATE,
    SLIP_DOWN_STATE,
    SLIP_UP_STATE,
    SCREW_ADJUST_FORCE_STATE,
    TRIGGER_ON_STATE,
    RELOAD_STATE,
    WAIT_TRIGGER_STATE,
    EMERGENCY_STATE,
};
void AUTO_OFF_FUNCTION(void) {
    auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    auto_shoot_cmd_send.Stella_speed = 0.0f;
    auto_shoot_cmd_send.Trigger_Location = 0.0f;
    auto_gimbal_cmd_send.yaw_speed = 0.0f;
    auto_shoot_delay_time=0;
    auto_shoot_cmd_flag=0;
    reload_start_flag=1;
    momokaSend_map->ready_flag = 0;
    momokaSend_map->magnet_flag=0;
    momokaSend_map->OnLine = Auto_Off;
    global_map->auto_debug_mode = Debug_on;
    current_state_AUTO_ON = 0;
    dart_number=4;
    batch_darts_fired=0;
    cd_shoot=0;
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
//     BuzzerPlaySequence(Auto_Off_beep, Auto_Off_beep,
// sizeof(Auto_Off_beep)/sizeof(Auto_Off_beep[0]),
// ALARM_LEVEL_HIGH);
    //yaw轴对准
    if (yaw_abnormol()){
        aim_output=0;
    }
    else{
        aim_output = PIDCalculate(&autoaim_pid, (float)slaveRecInfo->Pixel_Error, 0.0f+(float)global_map->Pixel_offset);
    }
    auto_gimbal_cmd_send.yaw_speed = 2.0f*aim_output;
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

static int d_Pixel_Error=0;
static int sum_Pixel_Error=0;
static int Pixel_Error_last=0;
static float d_Distance_dart=0;
static float Distance_dart=0;
static float K_offset=0;
static float force_dart_offset_ramdom=0;
static int flag_114514=1;
#define h_dart_2 625
#define K_force_dart_distancetoforce 0.1
#define pixel_to_x 0.005
#define K_dart_qujian_peishu 1.0f

static int flag_dart_ramdom_offset=1;
const int dichotomization[]={-1000,-100,-50,0,50,100,1000};
int get_segment(int val) {//二分查找
    int n = sizeof(dichotomization)/sizeof(int);
    int l = 0, r = n - 1;
    while (l <= r) {
        int m = (l + r) / 2;
        if (val < dichotomization[m]) r = m - 1;
        else l = m + 1;
    }
    return l;
}


static volatile float adv_CMD_data[4];
static volatile float CMD_Pixel_offset[4];

void AutoAim_Task() {
    SubGetMessage(shoot_cmd_sub, &shoot_cmd_recv);
    testtt = Sensor_force - global_map->adv_force_set;


    // d_Pixel_Error=slaveRecInfo->Pixel_Error-Pixel_Error_last;//用公式拟合，要标定的参数过多，舍弃
    // sum_Pixel_Error+=d_Pixel_Error;
    // K_offset=(pixel_to_x*sum_Pixel_Error)/sqrt((pixel_to_x*sum_Pixel_Error)*(pixel_to_x*sum_Pixel_Error)+h_dart_2);
    // d_Distance_dart=K_offset*(pixel_to_x*sum_Pixel_Error);
    // Distance_dart+=d_Distance_dart;
    // force_dart_offset_ramdom=K_force_dart_distancetoforce*Distance_dart;
    // if (force_dart_offset_ramdom>2)force_dart_offset_ramdom=2;
    // if (force_dart_offset_ramdom<-2)force_dart_offset_ramdom=-2;
    // Pixel_Error_last=slaveRecInfo->Pixel_Error;

    // if (flag_114514) {
    //     Pixel_Error_last=slaveRecInfo->Pixel_Error;
    //     flag_114514=0;
    // }
    d_Pixel_Error=slaveRecInfo->Pixel_Error-Pixel_Error_last;
    if (auto_gimbal_cmd_send.yaw_speed!=0) {
        sum_Pixel_Error+=d_Pixel_Error;
    }
    if (global_map->clear_sum_pixel_error_flag) {
        sum_Pixel_Error=0;
        global_map->clear_sum_pixel_error_flag=0;
    }
    force_dart_offset_ramdom=-K_dart_qujian_peishu*
        ((float)get_segment(sum_Pixel_Error)-  (  sizeof(dichotomization)/sizeof(int)/2.0f  ) );
    Pixel_Error_last=slaveRecInfo->Pixel_Error;


//老标参数在注释
    switch (dart_order) {
        case 1:
            adv_CMD_data[0]+=global_map->adv_dCMD_data/10000.0f;
            CMD_Pixel_offset[0]+=(float)global_map->CMD_dPixel_offset/10000.0f;
            global_map->adv_force_set =881.5f-1.5f-3.0+ adv_CMD_data[0];//883.0
            global_map->Pixel_offset = 115+(int)CMD_Pixel_offset[0];//130
            break;
        case 2:
            adv_CMD_data[1]+=global_map->adv_dCMD_data/10000.0f;
            CMD_Pixel_offset[1]+=(float)global_map->CMD_dPixel_offset/10000.0f;
            global_map->adv_force_set = 881.0f-4.0f-3.0-3.0+ adv_CMD_data[1];//884
            global_map->Pixel_offset = 100+(int)CMD_Pixel_offset[1];//115
            break;
        case 3:
            adv_CMD_data[2]+=global_map->adv_dCMD_data/10000.0f;
            CMD_Pixel_offset[2]+=(float)global_map->CMD_dPixel_offset/10000.0f;
            global_map->adv_force_set =880.0f-4.0f-3.0-3.0+ adv_CMD_data[2];//879
            global_map->Pixel_offset = 40+(int)CMD_Pixel_offset[2];//84
            break;
        case 4:
            adv_CMD_data[3]+=global_map->adv_dCMD_data/10000.0f;
            CMD_Pixel_offset[3]+=(float)global_map->CMD_dPixel_offset/10000.0f;
            if (cd_shoot==2) {
                global_map->adv_force_set = 1038+ adv_CMD_data[3];
                global_map->Pixel_offset = 140+CMD_Pixel_offset[3];
            }
            if (cd_shoot==0) {
                global_map->adv_force_set = 879-3.0-3.0+ adv_CMD_data[3];
                global_map->Pixel_offset = 74+CMD_Pixel_offset[3];//98
            }//880  好879

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