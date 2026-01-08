#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 18u // 当前为固定值,36字节
#define VISION_SEND_SIZE 36u

#pragma pack(1)
typedef enum {
    NO_FIRE = 0,
    AUTO_FIRE = 1,
    AUTO_AIM = 2
} Fire_Mode_e;

typedef enum {
    NO_TARGET = 0,
    TARGET_CONVERGING = 1,
    READY_TO_FIRE = 2
} Target_State_e;

typedef enum {
    NO_TARGET_NUM = 0,
    HERO1 = 1,
    ENGINEER2 = 2,
    INFANTRY3 = 3,
    INFANTRY4 = 4,
    INFANTRY5 = 5,
    OUTPOST = 6,
    SENTRY = 7,
    BASE = 8
} Target_Type_e;

typedef struct {
    Fire_Mode_e fire_mode;
    Target_State_e target_state;
    Target_Type_e target_type;

    float pitch;
    float yaw;
} Vision_Recv_s;

typedef enum {
//	COLOR_NONE = 0,
//	COLOR_BLUE = 1,
//	COLOR_RED = 2,
    ENEMY_COLOR_RED = 0,
    ENEMY_COLOR_BLUE = 1,
} Enemy_Color_e;

typedef enum {
    VISION_MODE_AIM = 0,
    VISION_MODE_SMALL_BUFF = 1,
    VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum {
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10 = 10,
    SMALL_AMU_15 = 15,
    BIG_AMU_16 = 16,
    SMALL_AMU_18 = 18,
    SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct {
    Enemy_Color_e enemy_color;
    Work_Mode_e work_mode;
    Bullet_Speed_e bullet_speed;

    float yaw;
    float pitch;
    float roll;
} Vision_Send_s;

typedef struct {
    uint8_t header;//0xff
    uint8_t mode; // 0 对面红  1 对面篮

    float roll;
    float yaw;
    float pitch;

    uint8_t game_status;
    uint16_t remaining_time;
    uint16_t sentry_blood;

    uint8_t ifenough_17mm_num;
    uint8_t whether2cruise;

    uint16_t outpost_hp;

    uint8_t free_data[/*7*/39];

    uint8_t checksum;
    uint8_t end;//0x0d
} SendPacket_t;

typedef struct {
    uint8_t mode;

    uint8_t game_status;
    uint16_t remaining_time;
    uint16_t sentry_blood;
    uint8_t ifenough_17mm_num;
    uint16_t outpost_hp;
} NavSet_t;

typedef  struct {
    uint8_t header; //= 0xff;
    uint8_t mode;
    float pitch;
    float yaw;
    float distance;
    int32_t target_id; // 前哨是8， 没目标位-1

    int32_t left_id; // 没有前哨 没目标为 -1
    float left_yaw;
    int32_t right_id;
    float right_yaw;

    float nav_vx;
    float nav_vy;
    float nav_wz;

    float angle_climb;
    uint8_t ifclimb;  // 爬坡信号
    float angle_attack_outpost;
    uint8_t if_attact_outpost;
    uint8_t free_data[6/*4*/];
    uint8_t checksum;
    uint8_t tail; //=0x0d;
} RecievePacket_t;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
//Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);
RecievePacket_t *VisionInit(void);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();

/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
//void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);
void VisionSetFlag(NavSet_t);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
//void VisionSetAltitude(float yaw, float pitch, float roll);
void VisionSetAltitude(float yaw, float pitch,float roll);


#endif // !MASTER_PROCESS_H