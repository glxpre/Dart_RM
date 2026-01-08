//
// Created by SJQ on 2023/12/19.
//

#include "power_meter.h"
#include "memory.h"
#include "stdlib.h"

static PowerMeterInstance *power_meter_instance = NULL; // 可以由app保存此指针

static void PowerMeterRxCallback(CANInstance *_instance)
{
    uint8_t *rxbuff;
    PowerMeter_Msg_s *Msg;
    rxbuff = _instance->rx_buff;
    Msg = &power_meter_instance->power_msg;
    Msg->vol = (uint16_t)(rxbuff[1] << 8 | rxbuff[0]);
    Msg->current = (uint16_t)(rxbuff[3] << 8 | rxbuff[2]);

}

PowerMeterInstance *PowerMeterInit(PowerMeter_Init_Config_s *PowerMeter_config)
{
    power_meter_instance = (PowerMeterInstance *)malloc(sizeof(PowerMeterInstance));
    memset(power_meter_instance, 0, sizeof(PowerMeterInstance));

    PowerMeter_config->can_config.can_module_callback = PowerMeterRxCallback;
    power_meter_instance->can_ins = CANRegister(&PowerMeter_config->can_config);
    return power_meter_instance;
}

float PowerMeterGet(PowerMeterInstance *instance)
{
    float power = instance->power_msg.current*instance->power_msg.vol/1e4;
    return power;
}

#include "power_meter.h"
