/*
 * @Author: szf
 * @Date: 2023-03-30 16:29:58
 * @LastEditTime: 2023-04-02 23:54:26
 * @LastEditors: szf
 * @Description: 
 * @FilePath: \ER-Upper-Stucture\UserCode\user_src\wtr_callback.c
 * @WeChat:szf13373959031
 */
/**
 * @description: 回调函数
 * @param {UART_HandleTypeDef} *huart
 * @author: szf
 * @date:
 * @return {void}
 */

#include "upper_start.h"
#include "wtr_callback.h"
#include "wtr_uart.h"
#include "wtr_mavlink.h"
#include "upper_operate_app.h"

int counter          = 0;
int test             = 0;
float w_speed        = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    test++;
    // MAVLINK消息
    if (huart->Instance == USART_MAVLINK) {
        wtrMavlink_UARTRxCpltCallback(huart, MAVLINK_COMM_0); // 进入mavlink回调
    }
    // DJI遥控器
    if (huart->Instance == USART_AS69)
    {
        AS69_Decode(); // AS69解码
    }
}


/**
 * @brief 接收到完整消息且校验通过后会调用这个函数。在这个函数里调用解码函数就可以向结构体写入收到的数据
 *
 * @param msg 接收到的消息
 * @return
 */
void wtrMavlink_MsgRxCpltCallback(mavlink_message_t *msg)
{

    switch (msg->msgid) {
        case MAVLINK_MSG_ID_CHASSIS_TO_UPPER:
            // id = 9 的消息对应的解码函数(mavlink_msg_xxx_decode)
            mavlink_msg_chassis_to_upper_decode(msg,&Msg_chassis_to_upper);
            break;
        case MAVLINK_MSG_ID_JOYSTICK_AIR:
            mavlink_msg_joystick_air_decode(msg,&Msg_joystick_air);
            break;
        // ......
        default:
            break;
    }
}
