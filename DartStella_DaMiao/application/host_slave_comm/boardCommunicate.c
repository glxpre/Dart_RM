/*
Created by CivilightEterna on 25-8-1.
*/

#include "boardCommunicate.h"
#include "bsp_log.h"
// #include "can_comm.h"
#include "Communication.h"
#include "daemon.h"
#include "dji_motor.h"
#include "gimbal.h"
#include "robot.h"
#include "shoot.h"
#include "usart.h"
// CANCommInstance *Cboard_can_comm; // 双板通信CAN comm
static momoka_Rec_info_t momokaRecInfo;
static momoka_Send_info_t momokaSendInfo;
static SlaveD_Rec_info_t SlaveDRecInfo;
static SlaveD_Send_info_t SlaveDSendInfo;
static uart_communicate_t *momokaComm;
static uart_communicate_t *slaveComm;

static DJIMotorInstance *Liyuu_map; // 头上丝杆电机
static DJIMotorInstance *yaw_map; // yaw电机

extern uint8_t dart_order;  //镖镖序号（外部定义）
static global_struct *global_map;

DynamicFilter* Sensor_Filter;
float Sensor_force;

momoka_Send_info_t *get_momokaSend() {
    return &momokaSendInfo;
}
momoka_Rec_info_t *get_momokaRec() {
    return &momokaRecInfo;
}
SlaveD_Rec_info_t *get_slaveRec(){
    return &SlaveDRecInfo;
};

void C_with_StellaSensorBoard_Init(void) {
    //TODO:这里是串口通信形式，debug完成
    uart_communicate_init_config_t config={
        .init_config = {
            .usart_handle = &huart7,
            .recv_buff_size = 128u,
        },
        .send_data_len = sizeof(momoka_Send_info_t),
        .recv_data_len = sizeof(momoka_Rec_info_t),
    };
    momokaComm = uartcommInit(&config);     //把接收到的解包数据cast为标准消息类型
    slaveComm = HostInit();

    global_map=get_global();  //获取全局发射标志
    Liyuu_map = (DJIMotorInstance *)get_Liyuu();
    // yaw_map = (DJIMotorInstance *)get_yaw();

    // DynamicFilter_Init(Sensor_Filter , (float)momokaRecInfo.Force_cur);

    // CANComm_Init_Config_s comm_config={
    //     .can_config = {
    //         .tx_id = 0x02,
    //         .rx_id = 0x01,
    //         .can_handle = &hcan1,
    //     },
    //     .recv_data_len = sizeof(momoka_Rec_info_t),
    //     .send_data_len = sizeof(momoka_Send_info_t),
    //     //.daemon_count = ?
    // };
    // Cboard_can_comm=CANCommInit(&comm_config);
;

    // momokaSendInfo = (momoka_Send_info_t *)malloc(sizeof(momoka_Send_info_t));
    //CANCommSend(StellaSensorBoardCANinstance,(uint8_t *)&Sender);
}

//主循环任务，发送与接收数据
void C_with_StellaSensorBoard_Task(void) {

    // momokaSendInfo.OnLine = 1;
    // momokaRecInfo = *(momoka_Rec_info_t *)CANCommGet(Cboard_can_comm);
    // uartcommSendData((uint8_t *)&momokaSendInfo);
    // momokaSendInfo.Liyuu_pos = ((int)Liyuu_map->measure.total_angle)/50;
    momokaSendInfo.dart_order_map = dart_order;
    momokaSendInfo.adv_Force = global_map->adv_force_set;
    momokaSendInfo.Liyuu_pos = (int)Liyuu_map->measure.total_angle;
    uartcommSendData((uint8_t *)&momokaSendInfo);

    SlaveDSendInfo.Shoot_Force_Init = 123;
    Slave_Send((uint8_t *)&SlaveDSendInfo);

    //数据更新检测
    momokaRecInfo=*(momoka_Rec_info_t *)uart_sensor_CommGet();
    if (momokaRecInfo.Force_cur > 1300) {
        Sensor_force = SensorFilter(momokaRecInfo.Force_cur);
    }
    else Sensor_force = momokaRecInfo.Force_cur;
    // Sensor_force = SensorFilter(momokaRecInfo.Force_cur);
    // Sensor_force = (int)DynamicFilter_Update(Sensor_Filter , (float)momokaRecInfo.Force_cur);


    if (slaveComm->update_flag) {
        SlaveDRecInfo=*(SlaveD_Rec_info_t *)uart_slave_CommGet();
    }

    // CANCommSend(Cboard_can_comm,(uint8_t *)&momokaSendInfo);
    // LOGINFO("[bsp_can] CAN Send");
    // if (Cboard_can_comm->update_flag) {
    //     momokaRecInfo = *(momoka_Rec_info_t *) CANCommGet(Cboard_can_comm);
    //     LOGINFO("[bsp_can] CAN get");
    // }

}

