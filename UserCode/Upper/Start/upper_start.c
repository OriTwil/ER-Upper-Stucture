/**
 * @Author: szf
 * @Date: 2023-02-22 10:59:01
 * @LastEditTime: 2023-02-22 19:27:40
 * @LastEditors: szf
 * @Description: ER
 * @FilePath: \ER\Usercode\user_src\upper_start.c
 * @WeChat:szf13373959031
 **/

#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "wtr_can.h"
#include "wtr_dji.h"
#include <math.h>
#include "main.h"
#include "upper_start.h"
#include "upper_pick_up.h"
#include "upper_servo.h"
#include "wtr_uart.h"
#include "wtr_calculate.h"
#include "wtr_callback.h"
#include "upper_operate_app.h"
#include "upper_communicate.h"

/**
 * @description: 创建线程
 * @author: szf
 * @date:
 * @return {void}
 */
void StartDefaultTask(void const *argument)
{

    // 初始化
    UpperStateInit();
    ServoInit();
    CommunicateInit();

    // 开启线程
    taskENTER_CRITICAL();
    CommunicateStart();
    ServoTaskStart();
    PickUpTaskStart();
    StateManagemantTaskStart();
    taskEXIT_CRITICAL();

    for (;;) {
        osDelay(1);
    }
}