//
// Created by SJQ on 2023/12/11.
//

#ifndef BASIC_FRAMEWORK_ECA8210_H
#define BASIC_FRAMEWORK_ECA8210_H

#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define EC_MOTOR_MX_CNT 4

#pragma pack(1)
typedef  struct
{
    uint8_t bug_info:5;       //错误信息
    uint8_t report_type:3;    //报文类别

    uint16_t angle_raw:16;    //输出端位置数值范围：0~65536 对应-12.5f~12.5f，单位 rad
    uint16_t speed_raw:12;    //输出端转速数值范围：0~4095 对应-18.0f~18.0f，单位 rad/s
    uint16_t current_raw:12;  //实际电流：0~4095 对应-30~30A
    uint8_t temperature_raw:8;//电机温度：反馈的数值数据类型为无符号 8 位，数值等于实际温度*2+50
}ECMotor_Report_t;
#pragma pack()
typedef  struct
{
    uint8_t mode:3;    //控制模式
    float32_t pos_target;       //错误信息
    uint16_t spd_target:15;    //输出端位置数值范围：0~65536 对应-12.5f~12.5f，单位 rad
    uint16_t max_current:12;    //输出端转速数值范围：0~4095 对应-18.0f~18.0f，单位 rad/s
    uint8_t ack_type:2;  //实际电流：0~4095 对应-30~30A

}ECMotor_PosCMD_t;
#pragma pack(1)
typedef  struct
{
    uint8_t ack_type:2;
    uint8_t reserve:3;
    uint8_t mode:3;    //控制模式


    float32_t spd_target;    //输出端速度指令  单位RPM
    uint16_t max_current;    //电流限幅  电机电流阈值数值解释：0~65536 对应 0~6553.6A，比例为10

}ECMotor_SpdCMD_t;

#pragma pack()

typedef struct // EC-A8120
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 当前编码器值
    float angle_single_round; // 单圈角度
    float speed_rads;         // speed rad/s
    float real_current;     // 实际电流
    uint8_t temperature;      // 温度,C°

    float total_angle;   // 总角度
    int32_t total_round; // 总圈数

    float feed_dt;
    uint32_t feed_dwt_cnt;
} ECMotor_Measure_t;

typedef struct
{
    ECMotor_Measure_t measure;

    Motor_Control_Setting_s motor_settings;

    float *other_angle_feedback_ptr; // 其他反馈来源的反馈数据指针
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;   // 速度前馈数据指针,可以通过此指针设置速度前馈值,或LQR等时作为速度状态变量的输入
    float *current_feedforward_ptr; // 电流前馈指针
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float pid_ref;

    Motor_Working_Type_e stop_flag; // 启停标志

    CANInstance *motor_can_ins;

    DaemonInstance *daemon;

} ECMotorInstance;

/**
 * @brief 初始化LK电机
 *
 * @param config 电机配置
 * @return LKMotorInstance* 返回实例指针
 */
ECMotorInstance *ECMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 设置参考值
 * @attention 注意此函数设定的ref是最外层闭环的输入,若要设定内层闭环的值请通过前馈数据指针设置
 *
 * @param motor 要设置的电机
 * @param ref 设定值
 */
void ECMotorSetRef(ECMotorInstance *motor, float ref);

/**
 * @brief 为所有EC电机计算pid/反转/模式控制,并通过bspcan发送电流值(发送CAN报文)
 *
 */
void ECMotorControl();

/**
 * @brief 停止EC电机,之后电机不会响应任何指令
 *
 * @param motor
 */
void ECMotorStop(ECMotorInstance *motor);

/**
 * @brief 启动EC电机
 *
 * @param motor
 */
void ECMotorEnable(ECMotorInstance *motor);

uint8_t ECMotorIsOnline(ECMotorInstance *motor);

#endif //BASIC_FRAMEWORK_ECA8210_H
