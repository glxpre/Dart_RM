/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "vision_transfer.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "remote_control.h"

#define RE_RX_BUFFER_SIZE 128u // 裁判系统接收缓冲区大小

static USARTInstance *vt_usart_instance; // 裁判系统串口实例
static DaemonInstance *vision_transfer_daemon;		  // 裁判系统守护进程
static referee_info_vt_t referee_info_vt;			  // 裁判系统数据
static VT_ctrl_t vt_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void JudgeReadVTData(uint8_t *buff)
{
    uint16_t judge_length; // 统计一帧数据长度
    if (buff == NULL)	   // 空数据包，则不作任何处理
        return;

    // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
    memcpy(&referee_info_vt.FrameHeader, buff, LEN_HEADER);

    // 判断帧头数据(0)是否为0xA5
    if (buff[SOF] == REFEREE_SOF)
    {
        // 帧头CRC8校验
        if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
        {
            // 统计一帧数据长度(byte),用于CR16校验
            judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
            {
                // 2个8位拼成16位int
                referee_info_vt.CmdID = (buff[6] << 8 | buff[5]);
                // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                // 第8个字节开始才是数据 data=7
                switch (referee_info_vt.CmdID)
                {
                    case  ID_custom_robot: //0x0302
                        memcpy(&referee_info_vt.ReceiveData, (buff + DATA_Offset), LEN__custom_robot);
                        break;
                    case  ID_remote_control: //0x0304
                        memcpy(&referee_info_vt.VisionTransfer, (buff + DATA_Offset), LEN_remote_control);
                        break;
                }
            }
        }
        // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
        if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info_vt.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
        { // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
            JudgeReadVTData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info_vt.FrameHeader.DataLength + LEN_TAIL);
        }
    }
}
static void vt_to_rc(void)
{
    // 鼠标解析
    vt_ctrl[TEMP].mouse.x = referee_info_vt.VisionTransfer.mouse_x; //!< Mouse X axis
    vt_ctrl[TEMP].mouse.y = referee_info_vt.VisionTransfer.mouse_y; //!< Mouse Y axis
    vt_ctrl[TEMP].mouse.press_l = referee_info_vt.VisionTransfer.left_button_down;                 //!< Mouse Left Is Press ?
    vt_ctrl[TEMP].mouse.press_r = referee_info_vt.VisionTransfer.right_button_down;                //!< Mouse Right Is Press ?

//  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    *(uint16_t *)&vt_ctrl[TEMP].key[KEY_PRESS] = referee_info_vt.VisionTransfer.keyboard_value;

    if (vt_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        vt_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = vt_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&vt_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (vt_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        vt_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = vt_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&vt_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = vt_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
    key_last = vt_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
    key_with_ctrl = vt_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
    key_with_shift = vt_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
    key_last_with_ctrl = vt_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
    key_last_with_shift = vt_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
// 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            vt_ctrl[TEMP].key_count[KEY_PRESS][i]++;
// 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            vt_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
// 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            vt_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&vt_ctrl[LAST], &vt_ctrl[TEMP], sizeof(VT_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

/*裁判系统串口接收回调函数,解析数据 */
static void VTRefereeRxCallback()
{
    DaemonReload(vision_transfer_daemon);
    JudgeReadVTData(vt_usart_instance->recv_buff);
    vt_to_rc();
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void VTRefereeLostCallback(void *arg)
{
    USARTServiceInit(vt_usart_instance);
    LOGWARNING("[rm_ref] lost referee vision data ");
}

/* 裁判系统通信初始化 */
VT_ctrl_t *VTRefereeInit(UART_HandleTypeDef *vt_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = VTRefereeRxCallback;
    conf.usart_handle = vt_usart_handle;
    conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
    vt_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
            .callback = VTRefereeLostCallback,
            .owner_id = vt_usart_instance,
            .reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
    };
    vision_transfer_daemon = DaemonRegister(&daemon_conf);

    return &vt_ctrl;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void VTRefereeSend(uint8_t *send, uint16_t tx_len)
{
    USARTSend(vt_usart_instance, send, tx_len, USART_TRANSFER_DMA);
    osDelay(115);
}
