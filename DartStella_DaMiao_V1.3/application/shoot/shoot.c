#include "shoot.h"

#include "auto_aim.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "ins_task.h"
#include "robot.h"
#include "servo_motor.h"
//
/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *Liyuu; // 拨盘电机
static DJIMotorInstance *Reload ; // 换弹电机
static ServoInstance *Trigger;
static DMMotorInstance *Stella;
// // static servo_instance *lid; 需要增加弹舱盖
//
static Publisher_t *shoot_pub;  //发射发布者
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Shoot_Ctrl_Cmd_s auto_shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;  //发射订阅者
static Subscriber_t *auto_shoot_cmd_sub;  //发射订阅者
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

static uint8_t reload_flag = 0;  //乒乓换弹标志
extern uint8_t dart_order;    //镖镖序号
static float last_reload_size;
float ori_angle ;
static global_struct *global_map;
// dwt定时,计算冷却用
// static float hibernate_time = 0,last_hibernate_time = 0, dead_time = 0;

void *get_Liyuu() {
    return Liyuu;
}
void ShootInit()
{
    Motor_Init_Config_s Stella_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x01,  //slave id  //0x01
            .rx_id = 0x02,  //MASTER ID  //0x02
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.0f,
                .Ki = 0.0f,
                .Kd = 0.00f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0.3f,
                //.MaxOut = 2,
            },
            .speed_PID = {
                .Kp = 0.25f,
                .Ki = 0.75f,
                .Kd = 0.001f,
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement |PID_Trapezoid_Intergral ,
                .IntegralLimit = 10,
                .MaxOut = 10,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type =  SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            //.feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = DM6006,
    };
    Stella=DMMotorInit(&Stella_config);

    Motor_Init_Config_s Liyuu_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 3
    },
    .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 12.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .DeadBand = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 500,
                .MaxOut = 80000,
            },
        .speed_PID = {
                .Kp = 1.5f,  //4000
                .Ki = 0.0f, //100
                .Kd = 0.0f, //0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,
                .MaxOut = 80000,
        },
    },
    .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            //.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
    },
    .motor_type = M2006
    };

    Liyuu = DJIMotorInit(&Liyuu_config);


    //换弹电机
    Motor_Init_Config_s Reload_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 4
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 32.0f,
                .Ki = 0.1f,
                .Kd = 0.0f,
                .DeadBand = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 500,
                .MaxOut = 80000,
            },
            .speed_PID = {
                .Kp = 8.0f,  //4000
                .Ki = 0.5f, //100
                .Kd = 0.001f, //0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,
                .MaxOut = 10000,
            },
        },
        .controller_setting_init_config = {
            // .feedforward_flag=SPEED_FEEDFORWARD,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            // .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
        .motor_type = GM6020,
        .motor_close_type = TOTAL_ANGLE,
        .motor_control_type = CURRENT_CONTROL,
};
    Reload = DJIMotorInit(&Reload_config);

    Servo_Init_Config_s trigger_config = {
        .htim = &htim1,
        .Channel = TIM_CHANNEL_1,
        .Servo_type = Servo270,
        .Servo_Angle_Type = Free_Angle_mode,
    };
    Trigger = ServoInit(&trigger_config);

    //换弹归零
    DJIMotorEnable(Reload);
    DJIMotorOuterLoop(Reload,ANGLE_LOOP);
    DJIMotorSetRef(Reload,reload_zeroSize);
    last_reload_size=reload_zeroSize;

    global_map=get_global();
    global_map->auto_shoot_set_flag = 1;

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    auto_shoot_cmd_sub = SubRegister("auto_shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

//
/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    //从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    //扳机角度设置
    Servo_Motor_FreeAngle_Set(Trigger,40+1.6*shoot_cmd_recv.Trigger_Location);
    //头上丝杆
    if (shoot_cmd_recv.liyuu_==Liyuu_DISABLE)
        DJIMotorStop(Liyuu);
    else if (shoot_cmd_recv.liyuu_==Speed_Control) {
        DJIMotorEnable(Liyuu);
        DJIMotorOuterLoop(Liyuu,SPEED_LOOP);
        DJIMotorSetRef(Liyuu,shoot_cmd_recv.Liyuu_speed);
    }
    //同步带处理
    if (shoot_cmd_recv.stella_mode==Lost_Control) {
        DMMotorStop(Stella);
        Stella->motor_controller.speed_PID.Iout=0.0f;//暂时这么处理，不太优雅，后续寻求别的方案
    }
    else if (shoot_cmd_recv.stella_mode==Enable) {
        DMMotorEnable(Stella);
        DMMotorOuterLoop(Stella,SPEED_LOOP);
        if (global_map->auto_shoot_begin_flag == 0) {  //依旧手动模式
            DMMotorSetRef(Stella,shoot_cmd_recv.Stella_speed);
        }
        if (global_map->auto_shoot_begin_flag == 1) {  //TODO：自动发射检测逻辑（半成品，后续待改进,使用状态机）
            if (global_map->auto_shoot_set_flag == 1) {  //触发拨轮开始信号
                ori_angle = Stella->measure.total_angle;
                global_map->auto_shoot_set_flag = 0;
                DMMotorSetRef(Stella , 13.0f);
            }
            if (Stella->measure.total_angle - ori_angle > 26.26) {
                DMMotorSetRef(Stella , -17.0f);
                global_map->auto_shoot_process_flag = 1;
            }
            if (Stella->measure.total_angle - ori_angle < 0 && global_map->auto_shoot_process_flag == 1) {
                DMMotorSetRef(Stella , 0);
                global_map->auto_shoot_process_flag = 0;
                global_map->auto_shoot_ready_flag = 1;
            }
        }
    }

    //换弹检测逻辑
    if (shoot_cmd_recv.reload_mode == Reload_disable) {
        DJIMotorStop(Reload);
    }
    if (shoot_cmd_recv.reload!=reload_flag) {
        dart_order += 1;
        if (dart_order > 4) {
            dart_order -= 4;
        }
        DJIMotorEnable(Reload);
        DJIMotorOuterLoop(Reload,ANGLE_LOOP);
        // if (last_reload_size+90.0f>360.0f)
        //     last_reload_size=last_reload_size+90.0f-360.0f;
        // else
            last_reload_size+=90.0f;
        DJIMotorSetRef(Reload,last_reload_size);
    }
    reload_flag=shoot_cmd_recv.reload;

}