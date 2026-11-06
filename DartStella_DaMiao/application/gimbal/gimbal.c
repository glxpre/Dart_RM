#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "dmmotor.h"

#define DEG_TO_RAD (PI / 180.0f)

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Subscriber_t *auto_gimbal_sub;                  // auto_cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static Gimbal_Ctrl_Cmd_s auto_gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU

typedef enum {
    ROLL_COMP_IDLE,
    ROLL_COMP_MOVING,
    ROLL_COMP_DONE,
    ROLL_COMP_FAILED
} RollCompState_e;

static RollCompState_e roll_comp_state = ROLL_COMP_IDLE;

static uint8_t last_roll_comp_request = 0;

static float roll_target_deg = 0.0f;  // 先通过实际校准确定
static float pitch_deg = 33.0f;
static float motor_target_deg;
static float motor_tolerance_deg = 2.0f;
static float motor_origin_deg;

// static float roll_comp_start_ms;//暂时不要
void *get_yaw() {
    return yaw_motor;
}

static void RollCompensationStart(void)
{
    motor_origin_deg = yaw_motor->measure.total_angle;

    if (gimba_IMU_data == NULL) {
        roll_comp_state = ROLL_COMP_FAILED;
        return;
    }

    float roll_diff_rad =
        (roll_target_deg - gimba_IMU_data->Roll) * DEG_TO_RAD;
    float pitch_rad = pitch_deg * DEG_TO_RAD;

    float yaw_offset_rad =
        atanf(tanf(roll_diff_rad) * tanf(pitch_rad));
    float yaw_offset_deg = yaw_offset_rad / DEG_TO_RAD;

    motor_target_deg = motor_origin_deg +yaw_offset_deg*36.0f;//M2006电机减速比

    // roll_comp_start_ms = DWT_GetTimeline_ms();
    roll_comp_state = ROLL_COMP_MOVING;
}



// static float roll_compensation()
// {
//     roll_now = gimba_IMU_data->Roll;
//     yaw_origin =gimba_IMU_data->Yaw;
//     roll_diff = (roll_ori - roll_now);//实测看正负, 这个roll_now由自己测出
//     yaw_offset =atanf(tanf(roll_diff)*tanf(pitch));
//     //todo:把yawoffset补偿到控制yaw的丝杆电机上
//     if (offset_flag)
//     {
//         DJIMotorEnable(yaw_motor);
//         DJIMotorOuterLoop(yaw_motor,ANGLE_LOOP);
//         DJIMotorSetRef(yaw_motor,yaw_offset);
//         if (abs(gimba_IMU_data->Yaw-(yaw_origin+yaw_offset))<= 0.2f ) offset_flag=0;
//     }
//
// }
void GimbalInit()
{
     gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // PITCH
        Motor_Init_Config_s yaw_config = {
                .can_init_config = {
                        .can_handle = &hcan1,
                        .tx_id = 4
                },
                .controller_param_init_config = {
                        .angle_PID = {
                                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                                .Kp = 17.0f,
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
                        // .angle_feedback_source = MOTOR_FEED,
                        // .speed_feedback_source = MOTOR_FEED,
                        .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
                        .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                        //.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
                },
                .motor_type = M2006 //yaw丝杆
        };

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    auto_gimbal_sub = SubRegister("auto_gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    SubGetMessage(auto_gimbal_sub , &auto_gimbal_cmd_recv);

    if (gimbal_cmd_recv.roll_comp_request && !last_roll_comp_request && roll_comp_state != ROLL_COMP_MOVING
        &&gimbal_cmd_recv.gimbal_mode != GIMBAL_ZERO_FORCE)
    {
        RollCompensationStart();
    }

    last_roll_comp_request = gimbal_cmd_recv.roll_comp_request;

    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE) {
        DJIMotorStop(yaw_motor);

        if (roll_comp_state == ROLL_COMP_MOVING) {
            roll_comp_state = ROLL_COMP_FAILED;
        }
    }

    if (roll_comp_state == ROLL_COMP_MOVING)
    {
        DJIMotorEnable(yaw_motor);
        DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
        DJIMotorSetRef(yaw_motor, motor_target_deg);

    //目前一旦误差小于 2° 就设为 DONE，下个周期进入原来的速度模式，有可能yaw_speed不为零，补偿后可能继续移动。回来实测一下
        if (fabsf(yaw_motor->measure.total_angle - motor_target_deg) < motor_tolerance_deg) {
            roll_comp_state = ROLL_COMP_DONE;
        }
    }// else if (DWT_GetTimeline_ms() - roll_comp_start_ms > timeout_ms) {//     roll_comp_state = ROLL_COMP_FAILED;}
    else
    {
        // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
        // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
        switch (gimbal_cmd_recv.gimbal_mode) {
            // 停止
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);
            DJIMotorStop(pitch_motor);
            break;
            // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_FREE_MODE: // 自由速控
            DJIMotorEnable(yaw_motor);
            DJIMotorOuterLoop(yaw_motor,SPEED_LOOP);
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw_speed);
            break;
        case GIMBAL_VISION_MODE:  //视觉自动模式
            DJIMotorEnable(yaw_motor);
            DJIMotorOuterLoop(yaw_motor,SPEED_LOOP);
            DJIMotorSetRef(yaw_motor, auto_gimbal_cmd_recv.yaw_speed);
        default:
            break;
        }
    }

        // 推送消息(暂无)
        PubPushMessage(gimbal_pub, (void *) &gimbal_feedback_data);
    }