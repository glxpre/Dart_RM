//
// Created by SJQ on 2023/12/19.
//

#ifndef BASIC_FRAMEWORK_POWER_METER_H
#define BASIC_FRAMEWORK_POWER_METER_H
#include "bsp_can.h"

#pragma pack(1)
typedef struct
{
    uint16_t vol; // 电压
    uint16_t current; // 电流
} PowerMeter_Msg_s;
#pragma pack()

/* 超级电容实例 */
typedef struct
{
    CANInstance *can_ins; // CAN实例
    PowerMeter_Msg_s power_msg; // 功率计反馈信息
} PowerMeterInstance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} PowerMeter_Init_Config_s;

/**
 * @brief 初始化功率计
 *
 * @param PowerMeter_config 功率计初始化配置
 * @return PowerMeterInstance* 功率计实例指针
 */
PowerMeterInstance *PowerMeterInit(PowerMeter_Init_Config_s *PowerMeter_config);
float PowerMeterGet(PowerMeterInstance *instance);

#endif //BASIC_FRAMEWORK_POWER_METER_H
