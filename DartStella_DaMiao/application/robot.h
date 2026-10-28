#ifndef ROBOT_H
#define ROBOT_H
#include <stdint.h>
#include <sys/types.h>
/* Robot利用robot_def.h中的宏对不同的机器人进行了大量的兼容,同时兼容了两个开发板(云台板和底盘板)的配置 */
/**
 * @brief 镖架全局状态标志位操作
 *
 */
typedef enum {
    Debug_on=0x01,
    Debug_ready,
}Debug_mode;
typedef struct {
    uint8_t fuck_them_all;
    uint8_t std_energy_pos_flag;  //储能丝杆标准位，需手动测量然后设定
    uint8_t auto_shoot_wait_flag; //等待丝杆到达标准位置
    uint8_t auto_shoot_begin_flag;  //全自动发射介入
    uint8_t auto_shoot_set_flag;  //初始位角度值
    uint8_t auto_shoot_process_flag;  //滑台拉到底标志位
    uint8_t auto_shoot_ready_flag;  //滑台复位标志位
    uint8_t auto_shoot_end_flag;  //发射完毕标志位
    uint8_t auto_routine_end_flag;  //自动逻辑结束标志位
    Debug_mode auto_debug_mode;

    float adv_force_preset;//镖架初始预紧力
    float adv_force_set; //镖镖预设发射拉力
    float adv_CMD_data;  //修改预设拉力值
    float adv_dCMD_data;
    int Pixel_offset;//像素值偏移修正
    int CMD_dPixel_offset;
    int CMD_Pixel_offset;
    uint8_t clear_sum_pixel_error_flag;
}global_struct;

global_struct *get_global(void);
void reset_global(void);
/**
 * @brief 机器人初始化,请在开启rtos之前调用.这也是唯一需要放入main函数的函数
 *
 */
void RobotInit();

/**
 * @brief 机器人任务,放入实时系统以一定频率运行,内部会调用各个应用的任务
 *
 */
void RobotTask();

#endif