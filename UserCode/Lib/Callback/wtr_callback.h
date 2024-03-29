/***
 * @Author: szf
 * @Date: 2023-02-22 12:05:56
 * @LastEditTime: 2023-02-22 12:25:39
 * @LastEditors: szf
 * @Description:
 * @FilePath: \ER\Usercode\user_inc\usercallback.h
 * @@WeChat:szf13373959031
 */
#ifndef _USERCALLBACK_H__
#define _USERCALLBACK_H__

#include "upper_start.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "wtr_calculate.h"
#include "wtr_can.h"
#include "wtr_dji.h"
#include "wtr_uart.h"
#include <math.h>
#include "main.h"
#include "wtr_mavlink.h"
#include "mavlink_msg_chassis_to_upper.h"
#include "wtr_callback.h"

extern int counter;
extern int test;
extern float w_speed;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void wtrMavlink_MsgRxCpltCallback(mavlink_message_t *msg);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif