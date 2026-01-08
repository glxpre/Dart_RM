/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID控制器定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "controller.h"
#include "memory.h"

/* ----------------------------下面是pid优化环节的实现---------------------------- */

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (fabsf(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (fabsf(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

//积分限幅
static void f_Integral_Limit(PIDInstance *pid) {
    // 计算累加后的积分
    float temp_Iout = pid->Iout + pid->ITerm;
    // 限制积分总和
    if (temp_Iout > pid->IntegralLimit) {
        pid->ITerm = pid->IntegralLimit - pid->Iout;
    } else if (temp_Iout < -pid->IntegralLimit) {
        pid->ITerm = -pid->IntegralLimit - pid->Iout;
    }
    // 确保ITerm不会导致超限
    pid->Iout += pid->ITerm;
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波（标准一阶低通滤波）
static void f_Derivative_Filter(PIDInstance *pid) {
    float alpha = pid->dt / (pid->Derivative_LPF_RC + pid->dt);
    pid->Dout = alpha * pid->Dout + (1 - alpha) * pid->Last_Dout;
}

// 输出滤波（同理）
static void f_Output_Filter(PIDInstance *pid) {
    if(pid->Output_LPF_RC <= 0) pid->Output_LPF_RC = 0;
    float alpha = pid->dt / (pid->Output_LPF_RC + pid->dt);
    pid->Output = alpha * pid->Output + (1 - alpha) * pid->Last_Output;
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid) {
    float original_output = pid->Output;
    // 限幅
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
    // 抗饱和处理：调整积分项
//    if (original_output != pid->Output) {
//        pid->Iout = pid->Output - pid->Pout - pid->Dout;
//    }
}

// 变比例
static void f_PID_Change_Proportion(PIDInstance *pid)
{
    float Err = abs(pid->Err) * 4;
    //float Err = abs(pid->Ref) / 0.7f;
    if(pid->ChanPmax - pid->ChanPmin< 1e-6f){
        pid->Kp = pid->kp_min;
    }
    else if(Err <= pid->ChanPmin){
        pid->Kp = pid->kp_max;
    }
    else if(Err <= pid->ChanPmax){//pid->critical_B + pid->critical_A){
        //pid->Kp = 0.1f + (0.6f - 0.1f) * (Err - pid->critical_B) / pid->critical_A;
        pid->Kp = pid->kp_min + (pid->kp_max - pid->kp_min) * (pid->ChanPmax - Err) / (pid->ChanPmax - pid->ChanPmin);
    }
    else
        pid->Kp = pid->kp_min;
}



// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /*Motor Blocked Handle*/
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
    if (pid->ERRORHandler.ERRORCount > 500) {
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
        // 堵转处理：清空积分，限制输出
        pid->Iout = 0;
        pid->Output = 0;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config) {
    memset(pid, 0, sizeof(PIDInstance));
    // 显式赋值每个配置项
    pid->Kp = config->Kp;
    pid->Ki = config->Ki;
    pid->Kd = config->Kd;
    pid->Improve = config->Improve;
    pid->MaxOut= abs(config->MaxOut);

    pid->kp_min = config->kp_min;
    pid->kp_max = config->kp_max;
    pid->ChanPmax = config->ChanPmax;
    pid->ChanPmin = config->ChanPmin;

//    pid->MinOut= config->MinOut;
    pid->DeadBand = config->DeadBand;
    pid->IntegralLimit = config->IntegralLimit;
    pid->Derivative_LPF_RC = config->Derivative_LPF_RC;
    pid->Output_LPF_RC = config->Output_LPF_RC;
    pid->CoefA = config->CoefA;
    pid->CoefB = config->CoefB;
    // 其他配置项赋值...
    DWT_GetDeltaT(&pid->DWT_CNT);
    pid->Last_Measure = 0; // 初始化为合理值或由首次测量更新
    pid->is_initialized = 0; // 添加初始化标志
    //新增前馈
    pid->Kf = config->Kf;
    pid->ff_type = config->ff_type;
    memset(pid->Accel_Body, 0, sizeof(pid->Accel_Body)); // 清空加速度
    pid->Last_Ref = 0; // 初始化为0或首次调用时更新
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // 首次调用初始化
    if (!pid->is_initialized) {
        pid->Last_Measure = measure;
        pid->Last_Err = ref - measure; // 初始化误差
        pid->is_initialized = 1;
    }

    // 堵转检测
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分
    if (pid->dt < 1e-6f) {
        pid->dt = 1e-6f; // 避免除以零
    }

    // 保存上次的测量值和误差,计算当前error
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 如果在死区外,则计算PID
    if (fabsf(pid->Err) > pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);
        if(pid->Improve & PID_Change_Proportion)
            f_PID_Change_Proportion(pid);

        // 前馈计算
        float feedforward = 0.0f;
        if (pid->Improve & PID_FeedForward) {
            switch (pid->ff_type) {
                case FF_Proportional:
                    feedforward = pid->Kf * ref;
                    break;
                case FF_Velocity:
                    if (pid->dt > 1e-6f) {
                        float dRef = (ref - pid->Last_Ref) / pid->dt;
                        feedforward = pid->Kf * dRef;
                    }
                    pid->Last_Ref = ref;
                    break;
                case FF_Acceleration: // 新增加速度前馈
                    // 假设Accel_Body[0]为云台轴向的加速度
                    feedforward = pid->Kf * pid->Accel_Body[0];
                    break;
                default:
                    break;
            }
        }

        pid->Iout += pid->ITerm;                         // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout + feedforward; // 计算输出

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;
    //pid->Last_Ref = pid->Ref;

    return pid->Output;
}