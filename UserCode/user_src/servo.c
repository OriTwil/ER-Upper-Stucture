/*
 * @Author: szf
 * @Date: 2023-03-11 12:51:38
 * @LastEditTime: 2023-04-02 15:11:56
 * @LastEditors: szf
 * @Description: 以固定频率进行伺服
 * @FilePath: \ER-Upper-Stucture\UserCode\user_src\servo.c
 * @WeChat:szf13373959031
 */

#include "servo.h"

void ServoTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    for (;;) {
        if (Pickup_state == Finished) {
            speedServo((Raw_Data.wheel-1024)*5, &hDJI[4]);
            positionServo(180, &hDJI[5]);
            positionServo(0, &hDJI[6]);
        }
        else if (Pickup_state == Ongoing) // 开始取环
        {
            switch (Pickup_mode) {
                case Motionless:    //初始取环三个机构
                    positionServo(0, &hDJI[4]);     //伸缩杆机构
                    positionServo(0, &hDJI[5]);     //经常崩坏齿轮的讨厌电机
                    positionServo(0, &hDJI[6]);     //爪子
                    break;
                case Overturn:      //翻转到爪子朝下
                    positionServo(0, &hDJI[4]);
                    positionServo(OverturnAngle - 100, &hDJI[5]);
                    positionServo(0, &hDJI[6]);
                    break;
                case Extend:
                    //positionServo(ExtendAngle + 200, &hDJI[4]);
                    speedServo((Raw_Data.wheel-1024)*5, &hDJI[4]);
                    positionServo(OverturnAngle, &hDJI[5]);
                    positionServo(0, &hDJI[6]);
                    break;
                case Claw_extend:
                    positionServo(ExtendAngle, &hDJI[4]);
                    positionServo(OverturnAngle, &hDJI[5]);
                    positionServo(ClawAngle - 10, &hDJI[6]);
                    break;
                case Retract:
                    positionServo(0, &hDJI[4]);
                    positionServo(OverturnAngle, &hDJI[5]);
                    positionServo(ClawAngle, &hDJI[6]);
                    break;
                case Overturn_back:
                    positionServo(2000, &hDJI[4]);
                    positionServo(180+50, &hDJI[5]);
                    positionServo(ClawAngle, &hDJI[6]);
                    break;
                case Claw_retract:
                    positionServo(2000, &hDJI[4]);
                    positionServo(180, &hDJI[5]);
                    positionServo(0+3, &hDJI[6]);
                    break;
            }
        }
        osDelayUntil(&PreviousWakeTime, 5);
    }
}

void ServoTaskStart(mavlink_controller_t *controldata)
{
    osThreadDef(servo, ServoTask, osPriorityBelowNormal, 0, 512);
    osThreadCreate(osThread(servo), NULL);
}
