//
// Created by 17946 on 25-9-13.
//

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "stdint.h"
#include "bsp_usart.h"
/****************************宏定义部分****************************/

#define UARTCOMM_SOF 0x05 // 起始字节,协议固定为0x05
#define UARTUSB_LEN_DATA 13  //上位机传输数据段长度为13字节

/****************************帧头****************************/

/* 帧头偏移 */
typedef enum
{
    uart_comm_SOF = 0,		 // 起始位
    uart_comm_LEN_HEADER = 1, // 帧头长
    uart_comm_LEN_TAIL = 1,  //帧尾长
} uart_comm_FrameHeaderOffset_e;

/* 帧头定义 */
typedef struct
{
    uint8_t SOF;
} uart_comm_xFrameHeader;
#pragma pack(1)
// 此结构体包含c板与sensor板间的通信相关信息
typedef struct {
    USARTInstance *Instance;
    uint8_t *recv_buff;
    //接收
    uint8_t unpacked_data[125];
    uint8_t packed_data[128];
    //发送
    uint8_t senddata[255];
    //长度
    uint8_t send_data_len; // 发送数据长度
    uint8_t recv_data_len;

    uint8_t recv_state;   // 接收状态,
    uint8_t update_flag;  // 数据更新标志位,当接收到新数据时,会将此标志位置1,调用CANCommGet()后会将此标志位置0
}uart_communicate_t;
#pragma pack()

typedef struct {
    //
    USART_Init_Config_s init_config;
    uint8_t send_data_len; // 发送数据长度
    uint8_t recv_data_len; // 接收数据长度
}uart_communicate_init_config_t;

typedef struct {
    float last_output;  // 上一次的输出值
    float min_alpha;    // 最小权重（决定稳态时的平滑度，越小越平滑，0.01~0.1）
    float max_alpha;    // 最大权重（决定突变时的跟随速度，越大越快跟上，0.5~1.0）
    float threshold;    // 动态判断阈值（根据你的传感器噪声幅值设定）
} DynamicFilter;

void uartcommSendData(uint8_t *senddata);
void Slave_Send(uint8_t *send);
uart_communicate_t  *uartcommInit(uart_communicate_init_config_t *config);
uart_communicate_t *HostInit();
static void uartcommReadData(uint8_t *buff);
void *uart_slave_CommGet();
void *uart_sensor_CommGet();

void DynamicFilter_Init(DynamicFilter* filter, float first_value);
float DynamicFilter_Update(DynamicFilter* filter, float new_value);
float SensorFilter(float input);
#endif //COMMUNICATION_H