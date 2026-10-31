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
#define AUTO_SHOOT_DELAY_MS 1000  // 延迟1s再进入下一次发射
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
static float tick_delay_time_end = 0;
static float tick_delay_time_start = 0;

static float testtt=0;
uint8_t dart_order = 1;    //镖镖序号
extern float Sensor_force;
static float ori_angle ; //自动发射开始时位置
static float std_energy_pos;  //储能丝杆标准位

static int current_state_MAIN = 0;
static int current_state_AUTO_ON=DEBUG_STATE_INDEX;
static int dart_number=4;
static int reload_start_flag=1;

static int tick_dart_station_state=1;
static int cd_shoot=0;
static int batch_darts_fired=0;
static int batch_size=3;

volatile float debug_stella_torque;

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
static float state_enter_tick;
static int static_enter_index=-1;
static uint8_t state_enter_dirty = 1;

static uint8_t state_enter_update()
{
    if (static_enter_index != current_state_AUTO_ON )
    {
        state_enter_tick = DWT_GetTimeline_ms();
        static_enter_index = current_state_AUTO_ON;
        return 1;
    }
    return 0;
}

static float state_elapsed_ms()
{
    float t = DWT_GetTimeline_ms() - state_enter_tick;
    if (t<0.0f)
    {
        t=0.0f;
    }
    return t;
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
    // PIDInit(&preforce_shooter_pid, &preforce_shooter_config);
    PIDInit(&pos_shooter_pid, &pos_shooter_config);

    auto_shoot_cmd_pub = PubRegister("auto_shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_cmd_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

    auto_gimbal_cmd_pub = PubRegister("auto_gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}



static AutoShootRequestSource_e auto_shoot_request_source = AUTO_SHOOT_REQUEST_NONE;
static Dart_station_state_enum dart_station_state = Dart_station_close;
static Dart_station_state_enum dart_station_last_state = Dart_station_close;


//将裁判系统发来的舱门数据进行更新
static void UpdateDartStationState(uint8_t referee_state) {
    switch (referee_state) {
        case(1) :
            if (dart_station_state != Dart_station_close) {
                dart_station_last_state = dart_station_state;
            }
            dart_station_state = Dart_station_close;
            break;
        case(0) :
            if (dart_station_state != Dart_station_open) {
                dart_station_last_state = dart_station_state;
            }
            dart_station_state = Dart_station_open;
            break;
        case(2) :
            if (dart_station_state != Dart_station_active_close2open &&
                dart_station_state != Dart_station_active_open2close) {
                dart_station_last_state = dart_station_state;
            }
            if (dart_station_state == Dart_station_open) {
                dart_station_state = Dart_station_active_open2close;
            }
            if (dart_station_state == Dart_station_close) {
                dart_station_state = Dart_station_active_close2open;
            }
            break;
    }
}

//遥控
static void UpdateRemoteShootRequest(void)
{
    if (global_map->auto_shoot_begin_flag && auto_shoot_request_source == AUTO_SHOOT_REQUEST_NONE) {
        auto_shoot_request_source = AUTO_SHOOT_REQUEST_REMOTE;
    }
}

//裁判系统
static void UpdateRefereeShootRequest(void)
{
    uint8_t station_state = referee_info.DartControlData.dart_station_state;

    if (referee_info.GameState.game_progress != 4) {
        tick_dart_station_state = station_state;
        tick_delay_time_start = 0.0f;
        tick_delay_time_end = 0.0f;
        tick_delay_time = 0.0f;
        return;
    }

    UpdateDartStationState(station_state);

    if (tick_dart_station_state == 1 && station_state == 2) {
        tick_delay_time_start = DWT_GetTimeline_ms();
        tick_delay_time_end = tick_delay_time_start;
    } else if (tick_dart_station_state == 2 && station_state == 2) {
        tick_delay_time_end = DWT_GetTimeline_ms();
    } else if (tick_dart_station_state == 2 && station_state == 0) {
        tick_delay_time = tick_delay_time_end - tick_delay_time_start;
        tick_delay_time_start = 0.0f;
        tick_delay_time_end = 0.0f;
    }

    tick_dart_station_state = station_state;

    if (dart_station_state == Dart_station_open &&
        dart_station_last_state == Dart_station_active_close2open &&
        tick_delay_time > 500.0f &&
        auto_shoot_request_source == AUTO_SHOOT_REQUEST_NONE) {
        auto_shoot_request_source = AUTO_SHOOT_REQUEST_REFEREE;
        tick_delay_time = 0.0f;
    }
}

//滑台和丝杆的准入条件，用在DEBUG_STATE里面
static uint8_t AutoShootReadyToStart(void)
{
    return momokaRec_map->slide_status == Slide_in_up
    // &&fabsf(pos_shooter_output) < pos_shooter_output_limit
    ;
}

int yaw_abnormol(){
    if (slaveRecInfo->Pixel_Error==-660)return 1;
    if (slaveRecInfo->Pixel_Error==0&&slaveRecInfo->Distance==0)return 1;
    return 0;
}

//准备阶段
void DEBUG_STATE(void) {

    if (dart_number <= 0) {
        return;
    }
    auto_shoot_cmd_send.Stella_speed = 1.0f;
    auto_shoot_cmd_send.Liyuu_speed = fabsf(shoot_cmd_recv.Liyuu_speed) > 0.0f  ? shoot_cmd_recv.Liyuu_speed: pos_shooter_output * 12.0f;
     //前哨转基地代码
   // if (cd_shoot && auto_shoot_request_source != AUTO_SHOOT_REQUEST_NONE &&
   //      slaveRecInfo->Pixel_Error == -660) {
   //      if (qwq_flag) {
   //          cd_shoot = 2;
   //          qwq_flag = 0;
   //          std_energy_pos -= 8800;
   //          pos_shooter_output_limit = 50.0f;
   //      }
   //      auto_gimbal_cmd_send.yaw_speed = -28000.0f;
   //  }

    //滑台在上面，记录电机初始位置
    if (momokaRec_map->slide_status == Slide_in_up) {
        ori_angle = Stella_map->measure.total_angle;
        auto_shoot_cmd_send.Stella_speed = 0.0f;
    }

    if (auto_shoot_request_source != AUTO_SHOOT_REQUEST_NONE && AutoShootReadyToStart()) {
        auto_shoot_cmd_send.Liyuu_speed = 0.0f;
        momokaSend_map->magnet_flag = 1;
        auto_shoot_request_source = AUTO_SHOOT_REQUEST_NONE;
        global_map->auto_shoot_begin_flag = 0;
        current_state_AUTO_ON = SLIP_DOWN_STATE_INDEX;
    }
}

//滑台下拉阶段
void SLIP_DOWN_STATE(void) {
    if (state_elapsed_ms() > 5000)
    {
        LOGERROR("[dart] slide DOWN timeout");
        current_state_AUTO_ON = EMERGENCY_STATE_INDEX;
        return;
    }
    auto_shoot_cmd_send.Stella_speed = -11.5f;
    if (Stella_map->measure.total_angle - ori_angle < -25.21f) {
        current_state_AUTO_ON=SLIP_UP_STATE_INDEX;
    }

}

//滑台上拉阶段
void SLIP_UP_STATE(void) {
    auto_shoot_cmd_send.Stella_speed = 13.5f;
    if (state_elapsed_ms() > 5000)
    {
        LOGERROR("[dart] slide UP timeout");
        current_state_AUTO_ON = EMERGENCY_STATE_INDEX;
        return;
    }
    if (Stella_map->measure.total_angle - ori_angle > -0.30f ) {
        current_state_AUTO_ON=SCREW_ADJUST_FORCE_STATE_INDEX;
        auto_shoot_cmd_send.Stella_speed = 0.0f;
    }
}

//力闭环校准
void SCREW_ADJUST_FORCE_STATE(void) {
    auto_shoot_cmd_send.Stella_speed = 0.0f;
    force_shooter_output = PIDCalculate(&force_shooter_pid, Sensor_force, global_map->adv_force_set);
    auto_shoot_cmd_send.Liyuu_speed = -force_shooter_output*17.0f;
    //超时保护
    if (state_elapsed_ms() > 15000)
    {
        LOGERROR("[dart] SCREW_ADJUST_FORCE_STATE timeout");
        current_state_AUTO_ON = EMERGENCY_STATE_INDEX;
        return;
    }
    //丝杆保护，防止力过大或过小
    if (Sensor_force >= 1500.0f || Sensor_force <= 300.0f) {
        current_state_AUTO_ON=EMERGENCY_STATE_INDEX;
        return;
    }
    //发射判断
    if (fabsf(testtt) < 1.0f) {
        current_state_AUTO_ON=TRIGGER_ON_STATE_INDEX;
    }
}

//扳机释放阶段
void TRIGGER_ON_STATE(void) {
    auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    auto_shoot_cmd_send.Trigger_Location=-80.0f;
    momokaSend_map->magnet_flag=0;
    current_state_AUTO_ON=RELOAD_STATE_INDEX;
}

//暂时不动了，之后会换新的换弹结构
void RELOAD_STATE(void) {
    if (reload_start_flag) {
        auto_shoot_delay_time = DWT_GetTimeline_ms();//todo:这里auto_shoot_delay_time与tick_delay_time重复了
        reload_start_flag=0;
    }  // 开始计时
    if (DWT_GetTimeline_ms() - auto_shoot_delay_time > AUTO_SHOOT_DELAY_MS && auto_shoot_delay_time!=0) {
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

//间隔连发模式,平常测试用
void WAIT_TRIGGER_STATE(void)
{
    auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    auto_shoot_cmd_send.Stella_speed = 0.0f;
    auto_shoot_cmd_send.Trigger_Location = 0.0f;
    auto_gimbal_cmd_send.yaw_speed=0;
    cd_shoot=1;
    if (auto_shoot_request_source != AUTO_SHOOT_REQUEST_NONE) {
        batch_darts_fired = 0;
        current_state_AUTO_ON = DEBUG_STATE_INDEX;
    }
}
void EMERGENCY_STATE(void) {
    auto_shoot_cmd_send.Liyuu_speed = 0.0f;
    auto_shoot_cmd_send.Stella_speed = 0.0f;
    auto_shoot_request_source = AUTO_SHOOT_REQUEST_NONE;
    global_map->auto_shoot_begin_flag = 0;
    momokaSend_map->magnet_flag = 0;
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
    auto_shoot_request_source = AUTO_SHOOT_REQUEST_NONE;
    reload_start_flag=1;
    momokaSend_map->ready_flag = 0;
    momokaSend_map->magnet_flag=0;
    momokaSend_map->OnLine = Auto_Off;
    global_map->auto_debug_mode = Debug_on;
    current_state_AUTO_ON = 0;
    dart_number= 4;
    batch_darts_fired= 0;
    cd_shoot= 0;
    static_enter_index = -1;

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

    UpdateRemoteShootRequest();
    UpdateRefereeShootRequest();
    // if (global_map->auto_shoot_wait_flag == 0 ){//|| abs(global_map->adv_force_set - momokaRec_map->Force_cur) > 2) {
    //     auto_shoot_cmd_send.Liyuu_speed = -pos_shooter_output*90.0f;
    //     // auto_shoot_cmd_send.Liyuu_speed = -(float)(global_map->adv_force_set - Sensor_force)*1440.0f;
    // }

    if (auto_aim_state_table_AUTO_ON[current_state_AUTO_ON] != NULL) {
        state_enter_update();
        auto_aim_state_table_AUTO_ON[current_state_AUTO_ON]();
    }
}

static void (*auto_aim_state_table_main[])(void)={AUTO_OFF_FUNCTION,AUTO_ON_FUNCTION};

static int d_Pixel_Error=0;
static int sum_Pixel_Error=0;
static int Pixel_Error_last=0;
static float force_dart_offset_ramdom=0;
#define h_dart_2 625
#define K_force_dart_distancetoforce 0.1
#define pixel_to_x 0.005
#define K_dart_qujian_peishu 1.0f

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

static float F_nat = 883.0f;//测老化后控制预紧力情况下（未老化时的预紧力）滑台拉到底部皮筋的力
static float F_nature[4] ={883.0f,883.0f,883.0f,883.0f};//这是皮筋在未老化之前命中目标的拉力
static float F_target[4] ={0.0f};//(F_nat+F_nature)/2.0f

void AutoAim_Task() {

    if (global_map->std_energy_pos_flag) {
        std_energy_pos = Liyuu_map->measure.total_angle / 50.0f;
        global_map->std_energy_pos_flag = 0;
    }

    debug_stella_torque = Stella_map->measure.torque;
    SubGetMessage(shoot_cmd_sub, &shoot_cmd_recv);
    testtt = Sensor_force - global_map->adv_force_set;

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


    for (int i = 0; i < 4; i++) {
        F_target[i] = (F_nature[i] + F_nat) / 2.0f;
    }

//老标参数在注释
    switch (dart_order) {
        case 1:
            adv_CMD_data[0]+=global_map->adv_dCMD_data/10000.0f;
            CMD_Pixel_offset[0]+=(float)global_map->CMD_dPixel_offset/10000.0f;
            global_map->adv_force_set =F_target[0] + adv_CMD_data[0];//883.0
            global_map->Pixel_offset = 40+(int)CMD_Pixel_offset[0];//130
            break;

        case 2:
            adv_CMD_data[1]+=global_map->adv_dCMD_data/10000.0f;
            CMD_Pixel_offset[1]+=(float)global_map->CMD_dPixel_offset/10000.0f;
            global_map->adv_force_set =F_target[1] + adv_CMD_data[1];//883.0
            global_map->Pixel_offset = 82+(int)CMD_Pixel_offset[1];//115
            break;

        case 3:
            adv_CMD_data[2]+=global_map->adv_dCMD_data/10000.0f;
            CMD_Pixel_offset[2]+=(float)global_map->CMD_dPixel_offset/10000.0f;
            // global_map->adv_force_set = 860.0+ adv_CMD_data[2];//879
            global_map->adv_force_set =F_target[2] + adv_CMD_data[2];//883.0
            global_map->Pixel_offset = 53+(int)CMD_Pixel_offset[2];//84
            break;

        case 4:
            adv_CMD_data[3]+=global_map->adv_dCMD_data/10000.0f;
            CMD_Pixel_offset[3]+=(float)global_map->CMD_dPixel_offset/10000.0f;
            if (cd_shoot==2) {
                global_map->adv_force_set = 1036+ adv_CMD_data[3];
                global_map->Pixel_offset = -18-20+CMD_Pixel_offset[3];
            }
            if (cd_shoot==1) {
                global_map->adv_force_set =F_target[3] + adv_CMD_data[3];//883.0
                global_map->Pixel_offset = 110+CMD_Pixel_offset[3];//98     74
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

    PubPushMessage(auto_gimbal_cmd_pub, (void *) &auto_gimbal_cmd_send);
    PubPushMessage(auto_shoot_cmd_pub, (void *) &auto_shoot_cmd_send);
}
