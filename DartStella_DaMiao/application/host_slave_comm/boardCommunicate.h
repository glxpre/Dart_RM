//
// Created by Civilight Eterna on 25-8-1.
//

#ifndef BOARDCOMMUNICATE_H
#define BOARDCOMMUNICATE_H

#include <stdbool.h>
#include <stdint.h>

#pragma pack(1)
//控制id
typedef enum {
    ShootNow=0x01,
    Force_init_change=0x02,
    Shoot_Force_change=0x03,
    Auto_Shoot=0x04,
    ShootWait=0x05,
}CmdID;
typedef enum {
    Loading_wait = 0x01,
    Loading_start = 0x02,
    Loading_ready = 0x03,
}slide_table_status;

// 此结构体包含c板与sensor板间的通信相关信息
typedef struct
{
    CmdID cmdID;    //控制id
    // uint8_t OnLine;//上线为1，掉线为0
    uint8_t return_flag;  //滑台回到上端
    uint8_t Liyuu_flag;
    int Force_cur;
} momoka_Rec_info_t;
typedef struct
{
    CmdID cmdID;    //控制id
    uint8_t OnLine;//上线为1，掉线为0
    int CMD_data;   //发射拉力修改值
    int adv_CMD_data; //预设拉力修改值
    // int Liyuu_pos; //头上丝杆位置
    uint8_t dart_order_map;
    uint8_t loaded_flag;
    uint8_t ready_flag; //检测是否已经归位，然后检测拉力值
} momoka_Send_info_t;

// 此结构体包含c板与上位机间的通信相关信息
typedef struct
{
    float Distance;  // 目标距离
    int Pixel_Error;   // 像素误差
                       // 计算公式：检测框的水平像素-理想点的水平像素
                       // 即偏左时<0，偏右时>0
    bool Debug_Mode;  // 是否启用调试模式（true: 启用，false: 不启用）
    int Shoot_Force_Set;  // 设定的发射拉力值
} SlaveD_Rec_info_t;
typedef struct
{
    int Shoot_Force_Init; //初始拉力值
    int Shoot_Force_Cur;  //当前拉力值
    float Shoot_vel;  //发射速度
    uint8_t filler;  //填充值，没啥用
} SlaveD_Send_info_t;
#pragma pack()

momoka_Send_info_t *get_momokaSend();
momoka_Rec_info_t *get_momokaRec();
SlaveD_Rec_info_t *get_slaveRec();
void momoka_RxCallback(void);
void C_with_StellaSensorBoard_Init(void);
void C_with_StellaSensorBoard_Task(void);


#endif //BOARDCOMMUNICATE_H