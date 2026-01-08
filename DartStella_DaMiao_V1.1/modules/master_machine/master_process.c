/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "crc16.h"

static RecievePacket_t recv_data;
static SendPacket_t send_data;
static DaemonInstance *vision_daemon_instance;


void VisionSetFlag(NavSet_t navset)
{
    send_data.mode = navset.mode;
    send_data.sentry_blood = navset.sentry_blood;
    send_data.game_status = navset.game_status;
    send_data.remaining_time = navset.remaining_time;
    send_data.ifenough_17mm_num = navset.ifenough_17mm_num;
    send_data.outpost_hp = navset.outpost_hp;
}


void VisionSetAltitude(float yaw, float pitch,float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}



/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance); // 喂狗
    get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
#include "crc8.h"
#include "bsp_dwt.h"

static uint8_t *vis_recv_buff; //接收到的原始数据

static void DecodeVision(uint16_t recv_len)  //作为回调函数调用
{

    if(vis_recv_buff[0]==0xff&&vis_recv_buff[/*31*/63] == 0x0d){
        if(crc_8(vis_recv_buff,/*30*/62)==vis_recv_buff[/*30*/62]){
            memcpy(&recv_data,vis_recv_buff,sizeof(recv_data));
        }
    }

}

/* 视觉通信初始化 */
RecievePacket_t *VisionInit(void)
{
//    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{

    static uint8_t send_buffer[sizeof(send_data)/*64*/]={0}; //todo 改成宏定义
    static uint32_t dicktick = 0;
    static uint8_t occupy = 1;

    send_data.header = 0xff;
    send_data.end = 0x0d;
//    send_data.outpost_hp = send_data_test.outpost_hp;
//    send_data.sentry_blood= send_data_test.sentry_blood;
//    send_data.game_status = send_data_test.game_status;
//    send_data.remaining_time = send_data_test.remaining_time;
//    send_data.mode = send_data_test.mode;
//    send_data.whether2cruise = send_data_test.whether2cruise;



    static float dt1 = 0,dt2 = 0;
    dt1 = DWT_GetTimeline_ms() - dt2;
    if(dt1 > 5000){
        if(occupy <  5) occupy++;
        else occupy = 1;
        dt2 = DWT_GetTimeline_ms();
    }


    send_data.whether2cruise = occupy;
    send_data.checksum = crc_8(&send_data.header,sizeof(send_data)-2);

    memcpy(send_buffer,&send_data,sizeof(send_data));
    USBTransmit(send_buffer, sizeof(send_buffer/*send_data*/));
}

#endif // VISION_USE_VCP
