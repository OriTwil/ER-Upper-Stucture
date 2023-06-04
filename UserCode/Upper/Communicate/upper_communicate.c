/*
 * @Author: szf
 * @Date: 2023-02-23 20:07:26
 * @LastEditTime: 2023-02-23 20:07:39
 * @LastEditors: szf
 * @Description:
 * @FilePath: \ER\Usercode\user_src\upper_communicate.c
 * @WeChat:szf13373959031
 */
/**
 * @file controller_data_sender.c
 * @author TITH (1023515576@qq.com)
 * @brief
 * @version 0.1
 * @date 2022-07-22
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "upper_communicate.h"

void CommunicateTask(void const *argument)
{
    while (1) {
        osDelay(10);
    }
}

void CommunicateStart()
{
    osThreadDef(CtrlSend, CommunicateTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(CtrlSend), NULL);
}

void CommunicateInit()
{
    wtrMavlink_BindChannel(&huart_mavlnik, MAVLINK_COMM_0);        // MAVLINK初始化
    HAL_UART_Receive_DMA(&huart_as69, JoyStickReceiveData, 18); // DMA接收AS69
}
