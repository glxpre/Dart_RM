#include "bsp_init.h"
#include "robot.h"

#include "boardCommunicate.h"
#include "robot_def.h"
#include "robot_task.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#endif
static global_struct global;

global_struct *get_global(void) {
    return &global;
}
void reset_global(void) {
    global.auto_shoot_begin_flag = 0;
    global.auto_shoot_end_flag = 0;
    global.auto_shoot_set_flag = 0;
    global.auto_shoot_process_flag = 0;
    global.auto_shoot_ready_flag = 0;
    global.fuck_them_all = 0;
}

void RobotInit()
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15, GPIO_PIN_SET);
    // LCD_Init();
    // LCD_Fill(0,0,LCD_W, LCD_H,BLACK);
    // LCD_ShowString(20, 12,(uint8_t *)"RoboMaster Init...", LIGHTBLUE, BLACK, 24, 0);
    __HAL_TIM_SET_PRESCALER(&htim12,24-1);
    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,1880);

    //HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);//pwm开始输出
    // BuzzerInit();
    __disable_irq();


    BSPInit();

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDInit();
    GimbalInit();
    ShootInit();
    C_with_StellaSensorBoard_Init();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    // ChassisInit();
#endif

    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDTask();
    GimbalTask();
    ShootTask();
    C_with_StellaSensorBoard_Task();

#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    // ChassisTask();
#endif

}