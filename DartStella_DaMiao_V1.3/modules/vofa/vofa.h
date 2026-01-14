/*
 * @Descripttion: 
 * @version: 
 * @Author: Chenfu
 * @Date: 2022-12-05 12:39:18
 * @LastEditTime: 2022-12-05 13:37:36
 */
#ifndef VOFA_H
#define VOFA_H
#include <stdint.h>
#include "bsp_usart.h"
#include "usart.h"


void vofa_justfloat_output(float *data, uint8_t num , UART_HandleTypeDef *huart);
void ANODT_SendF1(int32_t Angle,int32_t speed_rpm,int32_t Angle_target,int32_t speed_target);
#endif // !1#define 