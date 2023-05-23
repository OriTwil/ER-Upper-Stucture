/*** 
 * @Author: szf
 * @Date: 2023-03-11 12:51:24
 * @LastEditTime: 2023-03-11 16:08:55
 * @LastEditors: szf
 * @Description: 
 * @FilePath: \ER-Upper-Stucture\UserCode\user_inc\servo.h
 * @@WeChat:szf13373959031
 */
#ifndef SERVO_H
#define SERVO_H

#include "pick_up.h"
#include "user_main.h"

void ServoTask(void const *argument);
void ServoTaskStart(mavlink_controller_t *controldata);

#endif