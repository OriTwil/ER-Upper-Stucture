/*** 
 * @Author: szf
 * @Date: 2023-02-22 11:56:47
 * @LastEditTime: 2023-03-30 16:44:04
 * @LastEditors: szf
 * @Description: 
 * @FilePath: \ER-Upper-Stucture\UserCode\user_inc\usermain.h
 * @@WeChat:szf13373959031
 */


#ifndef USERMAIN_H
#define USERMAIN_H

#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "user_calculate.h"
#include "wtr_can.h"
#include "wtr_dji.h"
#include <math.h>
#include "main.h"
#include "wtr_mavlink.h"
#include "user_config.h"
#include "servo.h"
#include "pick_up.h"
#include "wtr_uart.h"
#include "user_callback.h"

extern mavlink_controller_t ControllerData;

#endif