//
// Created by SJQ on 2023/12/11.
//

#include "ECA8210.h"

ECMotor_Report_t ECMotor_Report;
ECMotor_SpdCMD_t ECMotor_SpdCMD;
ECMotor_PosCMD_t ECMotor_PosCMD;
static uint8_t idx;
static ECMotorInstance *ecmotor_instance[EC_MOTOR_MX_CNT] = {NULL};

static void swap ( uint8_t *a ,uint8_t *b )
{
    uint8_t temp = *a;
    *a = *b;
    *b = temp;
}
/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 */
static void ECMotorDecode(CANInstance *_instance)
{
    ECMotorInstance *motor = (ECMotorInstance *)_instance->id; // 通过caninstance保存的father id获取对应的motorinstance
    ECMotor_Measure_t *measure = &motor->measure;
    uint8_t *rx_buff = _instance->rx_buff;

    swap(rx_buff+1,rx_buff+2);// 输出端位置  高位在前

    memcpy(&ECMotor_Report,rx_buff,sizeof(ECMotor_Report));
    //高位在前  这两个12位数据需要做特殊处理
    ECMotor_Report.speed_raw = (uint16_t)(rx_buff[3]<<4|(rx_buff[4]>>4));
    ECMotor_Report.current_raw = (uint16_t)(rx_buff[4]<<8|(rx_buff[5]));

    DaemonReload(motor->daemon); // 喂狗
    measure->feed_dt = DWT_GetDeltaT(&measure->feed_dwt_cnt);

    measure->angle_single_round = (float)(ECMotor_Report.angle_raw-32768) * 0.0003814697;
    measure->speed_rads = (float)(ECMotor_Report.speed_raw-2048) * 0.008789;
    measure->real_current = (float)(ECMotor_Report.current_raw-2048) * 0.014648;
    measure->temperature = (ECMotor_Report.temperature_raw - 50) /2;
}

static void ECMotorLostCallback(void *motor_ptr)
{
    ECMotorInstance *motor = (ECMotorInstance *)motor_ptr;
    LOGWARNING("[ECMotor] motor lost, id: %d", motor->motor_can_ins->tx_id);
}

ECMotorInstance *ECMotorInit(Motor_Init_Config_s *config)
{
    ECMotorInstance *motor = (ECMotorInstance *)malloc(sizeof(ECMotorInstance));
    motor = (ECMotorInstance *)malloc(sizeof(ECMotorInstance));
    memset(motor, 0, sizeof(ECMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.id = motor;
    config->can_init_config.can_module_callback = ECMotorDecode;
    config->can_init_config.rx_id = config->can_init_config.tx_id;
    config->can_init_config.tx_id = config->can_init_config.tx_id;
    motor->motor_can_ins = CANRegister(&config->can_init_config);

    ECMotorEnable(motor);

    DWT_GetDeltaT(&motor->measure.feed_dwt_cnt);
    ecmotor_instance[idx++] = motor;

    Daemon_Init_Config_s daemon_config = {
            .callback = ECMotorLostCallback,
            .owner_id = motor,
            .reload_count = 5, // 50ms
    };
    motor->daemon = DaemonRegister(&daemon_config);

    return motor;
}



void ECMotorControl()
{
    float pid_measure, pid_ref;
    int16_t set;
    ECMotorInstance *motor;
    ECMotor_Measure_t *measure;
    Motor_Control_Setting_s *setting;

    for (size_t i = 0; i < idx; ++i)
    {
        motor = ecmotor_instance[i];
        measure = &motor->measure;
        setting = &motor->motor_settings;
        pid_ref = motor->pid_ref;

        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_angle_feedback_ptr;
            else
                pid_measure = measure->real_current;
            pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor->speed_feedforward_ptr;
        }

        if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_speed_feedback_ptr;
            else
                pid_measure = measure->speed_rads;
            pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
                pid_ref += *motor->current_feedforward_ptr;
        }

        if (setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor->current_PID, measure->real_current, pid_ref);
        }

        set = pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1;

        ECMotor_SpdCMD.mode = 0b010;
        ECMotor_SpdCMD.reserve = 0b000;
        ECMotor_SpdCMD.ack_type = 0b01;
        ECMotor_SpdCMD.spd_target = set;
        ECMotor_SpdCMD.max_current = 10;
        // 这里随便写的,为了兼容多电机命令.后续应该将tx_id以更好的方式表达电机id,单独使用一个CANInstance,而不是用第一个电机的CANInstance
        //memcpy(sender_instance->tx_buff + (motor->motor_can_ins->tx_id - 0x280) * 2, &set, sizeof(uint16_t));
        CANSetDLC(motor->motor_can_ins,7);
        memcpy(motor->motor_can_ins->tx_buff,&ECMotor_SpdCMD,sizeof(ECMotor_SpdCMD));
        //发送数据高位在前！！ 交换一下顺序
        swap(motor->motor_can_ins->tx_buff+1,motor->motor_can_ins->tx_buff+4);
        swap(motor->motor_can_ins->tx_buff+2,motor->motor_can_ins->tx_buff+3);
        swap(motor->motor_can_ins->tx_buff+5,motor->motor_can_ins->tx_buff+6);

        //memcpy(motor->motor_can_ins->tx_buff+1,&ECMotor_SpdCMD.spd_target,sizeof(float32_t));

        if (motor->stop_flag == MOTOR_STOP)
        { // 若该电机处于停止状态,直接将发送buff置零
            memcpy(motor->motor_can_ins->tx_buff,0,sizeof(ECMotor_SpdCMD));
        }

        CANTransmit(motor->motor_can_ins,0.2);
    }


}

void ECMotorStop(ECMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void ECMotorEnable(ECMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void ECMotorSetRef(ECMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

uint8_t ECMotorIsOnline(ECMotorInstance *motor)
{
    return DaemonIsOnline(motor->daemon);
}
