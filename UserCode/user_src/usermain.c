/**
 * @Author: szf
 * @Date: 2023-02-22 10:59:01
 * @LastEditTime: 2023-02-22 19:27:40
 * @LastEditors: szf
 * @Description: ER
 * @FilePath: \ER\Usercode\user_src\usermain.c
 * @WeChat:szf13373959031
 **/

#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "Caculate.h"
#include "wtr_can.h"
#include "DJI.h"
#include <math.h>
#include "main.h"
#include "usermain.h"
#include "ctrl_data_sender.h"
#include "beep.h"
#include "pick_up.h"
#include "servo.h"
#include "wtr_uart.h"
#include "usercallback.h"

// 变量定义
mavlink_controller_t ControllerData = {0};

/**
 * @description: 创建线程
 * @author: szf
 * @date:
 * @return {void}
 */
void StartDefaultTask(void const *argument)
{
    //初始化
    wtrMavlink_BindChannel(&huart8, MAVLINK_COMM_0);// MAVLINK初始化
    CtrlDataSender_Init(&huart1, MAVLINK_COMM_1); // 遥控器初始化
    HAL_UART_Receive_DMA(&huart1, JoyStickReceiveData, 18); // DMA接收AS69

    //开启线程
	// CtrlDataSender_Start(&ControllerData);// 遥控器线程
    ServoTaskStart(&ControllerData);
    PickUpTaskStart(&ControllerData);// 取环线程
    FireTaskStart(&ControllerData);// 射换线程
    
    for (;;) {
        osDelay(1);
    }
}