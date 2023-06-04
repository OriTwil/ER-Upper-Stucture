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

#include "upper_pick_up.h"
#include "upper_start.h"

void ServoTask(void const *argument);
void ServoTaskStart();
void ServoInit();

#endif