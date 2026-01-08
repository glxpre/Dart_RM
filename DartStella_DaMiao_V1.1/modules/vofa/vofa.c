/*
 * @Descripttion: 
 * @version: 
 * @Author: Chenfu
 * @Date: 2022-12-05 12:39:07
 * @LastEditTime: 2022-12-05 14:15:53
 */
#include "vofa.h"
#include "usbd_cdc_if.h"

typedef union
{
    float float_t;
    uint8_t uint8_t[4];
} send_float;

/*VOFA浮点协议*/
void vofa_justfloat_output(float *data, uint8_t num , UART_HandleTypeDef *huart )
{
    static uint8_t i = 0;
    send_float temp[num];			//定义缓冲区数组
    uint8_t send_data[4 * num + 4]; //定义通过串口传出去的数组，数量是所传数据的字节数加上4个字节的尾巴
    for (i = 0; i < num; i++)
    {
        temp[i].float_t = data[i]; //将所传数据移到缓冲区数组
    }
    for (i = 0; i < num; i++)
    {
        send_data[4 * i] = temp[i].uint8_t[0];
        send_data[4 * i + 1] = temp[i].uint8_t[1];
        send_data[4 * i + 2] = temp[i].uint8_t[2];
        send_data[4 * i + 3] = temp[i].uint8_t[3]; //将缓冲区数组内的浮点型数据转成4个字节的无符号整型，之后传到要通过串口传出的数组里
    }
    send_data[4 * num] = 0x00;
    send_data[4 * num + 1] = 0x00;
    send_data[4 * num + 2] = 0x80;
    send_data[4 * num + 3] = 0x7f; //加上协议要求的4个尾巴

    //HAL_UART_Transmit(huart, (uint8_t *)send_data, 4 * num + 4, 100);
    CDC_Transmit_HS((uint8_t *)send_data,4 * num + 4);
}

#define BYTE0(dwTemp) (*(char*)(&dwTemp))
#define BYTE1(dwTemp) (*((char*)(&dwTemp)+1))
#define BYTE2(dwTemp) (*((char*)(&dwTemp)+2))
#define BYTE3(dwTemp) (*((char*)(&dwTemp)+3))

uint8_t DataSendBuf[100];
//匿名上位机
void ANODT_SendF1(int32_t Angle,int32_t speed_rpm,int32_t Angle_target,int32_t speed_target)//F1灵活格式帧
{
    uint8_t cnt=0;
    DataSendBuf[cnt++]=0xAA;  //帧头
    DataSendBuf[cnt++]=0xFF;  //目标地址
    DataSendBuf[cnt++]=0xF1;  //功能码
    DataSendBuf[cnt++]=16;  //数据长度

    DataSendBuf[cnt++]=BYTE0(Angle);
    DataSendBuf[cnt++]=BYTE1(Angle);
    DataSendBuf[cnt++]=BYTE2(Angle);
    DataSendBuf[cnt++]=BYTE3(Angle);

    DataSendBuf[cnt++]=BYTE0(speed_rpm);
    DataSendBuf[cnt++]=BYTE1(speed_rpm);
    DataSendBuf[cnt++]=BYTE2(speed_rpm);
    DataSendBuf[cnt++]=BYTE3(speed_rpm);

    DataSendBuf[cnt++]=BYTE0(Angle_target);
    DataSendBuf[cnt++]=BYTE1(Angle_target);
    DataSendBuf[cnt++]=BYTE2(Angle_target);
    DataSendBuf[cnt++]=BYTE3(Angle_target);

    DataSendBuf[cnt++]=BYTE0(speed_target);
    DataSendBuf[cnt++]=BYTE1(speed_target);
    DataSendBuf[cnt++]=BYTE2(speed_target);
    DataSendBuf[cnt++]=BYTE3(speed_target);

    uint8_t sc=0;  //和校验
    uint8_t ac=0;  //附加校验
    for(uint8_t i=0;i<DataSendBuf[3]+4;i++)
    {
        sc+=DataSendBuf[i];
        ac+=sc;
    }
    DataSendBuf[cnt++]=sc;
    DataSendBuf[cnt++]=ac;

//	for(uint8_t i=0;i<cnt;i++)
//		HAL_UART_Transmit(&huart6,&DataSendBuf[i],1,100);
    CDC_Transmit_HS(DataSendBuf, cnt);

}
