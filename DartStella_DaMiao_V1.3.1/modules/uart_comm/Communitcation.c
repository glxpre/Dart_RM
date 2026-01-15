//
// Created by 17946 on 25-9-13.
//
#include <string.h>
#include "bsp_log.h"
#include "Communication.h"
#include "crc8.h"
#include "boardCommunicate.h"
#include "bsp_usb.h"
// #include "daemon.h"
#include "master_process.h"

static uart_comm_xFrameHeader header;

//通信结构体
uart_communicate_t momokaComm;
uart_communicate_t slaveComm;

static void uartCommResetRx()
{
    // 当前已经收到的buffer清零
    memset(momokaComm.packed_data, 0, 128);
    memset(slaveComm.packed_data, 0, 128);
    momokaComm.recv_state = 0;   // 接收状态重置
}

//双板通信接收回调
void uart_comm_callback_1() {
    LOGINFO("UART1 GET");
    uartcommReadData(momokaComm.Instance->recv_buff);
    momokaComm.update_flag = 1;
}
//上位机通信接收回调
void uart_comm_callback_2(uint16_t len) {
    (void)len;
    LOGINFO("UARTUSB GET");
    uartcommReadData(slaveComm.recv_buff);
    slaveComm.update_flag = 1;
}


//双板通信、上位机接收数据解算
static void uartcommReadData(uint8_t *buff ) {
    // uint16_t judge_length; // 统计一帧数据长度
    if (buff == NULL)	  // 空数据包，则不作任何处理
        return;
    //双板通信解包
    if (buff == momokaComm.Instance->recv_buff) {
        memcpy(momokaComm.packed_data , buff , momokaComm.recv_data_len+uart_comm_LEN_HEADER+uart_comm_LEN_TAIL);
        if (buff[uart_comm_SOF] == UARTCOMM_SOF)
        {
            uint8_t crc8 = crc_8(momokaComm.packed_data+uart_comm_LEN_HEADER, momokaComm.recv_data_len);
            if (momokaComm.packed_data[momokaComm.recv_data_len+uart_comm_LEN_HEADER] == crc8) {
                LOGINFO("CRC8 PASS");
                memcpy(momokaComm.unpacked_data , buff+uart_comm_LEN_HEADER , momokaComm.recv_data_len);
                uartCommResetRx();
            }
        }
    }
    //上位机解包
    if (buff == slaveComm.recv_buff) {
        memcpy(slaveComm.packed_data , buff , UARTUSB_LEN_DATA+uart_comm_LEN_HEADER+uart_comm_LEN_TAIL);
        if (buff[uart_comm_SOF] == UARTCOMM_SOF)
        {
            uint8_t crc8 = crc_8(slaveComm.packed_data+uart_comm_LEN_HEADER, UARTUSB_LEN_DATA);
            if (slaveComm.packed_data[UARTUSB_LEN_DATA+uart_comm_LEN_HEADER] == crc8) {
                LOGINFO("CRC8 PASS");
                memcpy(slaveComm.unpacked_data , buff+uart_comm_LEN_HEADER , UARTUSB_LEN_DATA);
                uartCommResetRx();
            }
        }
    }
}

//双板通信发送函数
void uartcommSendData(uint8_t *senddata) {
    //数据
    memcpy(momokaComm.senddata+uart_comm_LEN_HEADER , senddata , momokaComm.send_data_len);
    //校验
    uint8_t crc8 = crc_8(senddata, momokaComm.send_data_len);
    momokaComm.senddata[uart_comm_LEN_HEADER+momokaComm.send_data_len] = crc8;
    //阻塞发送
    USARTSend(momokaComm.Instance, momokaComm.senddata, momokaComm.send_data_len+uart_comm_LEN_HEADER+1, USART_TRANSFER_BLOCKING);
}
//上位机通信发送函数
void Slave_Send(uint8_t *send)
{
    //数据
    memcpy(slaveComm.senddata+uart_comm_LEN_HEADER , send , UARTUSB_LEN_DATA);
    //校验
    uint8_t crc8 = crc_8(send, UARTUSB_LEN_DATA);
    slaveComm.senddata[uart_comm_LEN_HEADER+UARTUSB_LEN_DATA] = crc8;

    USBTransmit(slaveComm.senddata, 15);
}


//双板通信结构体初始化，传入回调函数入口、发送数据长度、接收数据长度
uart_communicate_t  *uartcommInit(uart_communicate_init_config_t *config)
{
    config->init_config.module_callback=uart_comm_callback_1;
    momokaComm.Instance = USARTRegister(&config->init_config);

    momokaComm.send_data_len = config->send_data_len;
    momokaComm.recv_data_len = config->recv_data_len;

    momokaComm.senddata[0]=UARTCOMM_SOF;

    return &momokaComm;   //初始化后返回通信结构体的指针
}

//上位机通信结构体初始化（虽然没用普通串口，但是还是用了通用的串口结构体，只是没有初始化HandleTypeDefine）
uart_communicate_t *HostInit()
{
    USB_Init_Config_s conf = {.rx_cbk = uart_comm_callback_2};
    slaveComm.recv_buff = USBInit(conf);
    slaveComm.senddata[0]=UARTCOMM_SOF;

    return &slaveComm;  //初始化后返回通信结构体的指针
}

void *uart_sensor_CommGet()
{
    momokaComm.update_flag = 0; // 读取后将更新flag置为0
    return &momokaComm.unpacked_data;
}
void *uart_slave_CommGet()
{
    slaveComm.update_flag = 0; // 读取后将更新flag置为0
    return &slaveComm.unpacked_data;
}

