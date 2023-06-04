/*
 * @Author: szf
 * @Date: 2023-03-11 12:51:38
 * @LastEditTime: 2023-04-02 15:11:56
 * @LastEditors: szf
 * @Description: 以固定频率进行伺服
 * @FilePath: \ER-Upper-Stucture\UserCode\user_src\upper_servo.c
 * @WeChat:szf13373959031
 */

#include "upper_servo.h"
#include "upper_operate_app.h"
#include "wtr_vesc.h"

SERVO_REF_FIRE temp_fire_ref;

void ServoTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    for (;;) {
         
        xSemaphoreTake(Fire_ref.xMutex_servo_fire, (TickType_t)10);
        // 推环、递环、Pitch、Yaw轴电机的伺服
        // 射环两个电机伺服拷贝
        temp_fire_ref = ReadServoRefFire(&Fire_ref);
        xSemaphoreGive(Fire_ref.xMutex_servo_fire);
        positionServo(temp_fire_ref.position_servo_ref_push,&hDJI[Motor_Push_id]);
        positionServo(temp_fire_ref.position_servo_ref_pass,&hDJI[Motor_pass_id]);
        positionServo(temp_fire_ref.position_servo_ref_pitch,&hDJI[Motor_Pitch_id]);
        positionServo(temp_fire_ref.position_servo_ref_yaw,&hDJI[Motor_Yaw_id]);

        // 取环三个电机的伺服
        xSemaphoreTake(Pickup_ref.xMutex_servo_pickup, (TickType_t)10);
        positionServo(Pickup_ref.position_servo_ref_claw,&hDJI[Motor_Claw_id]);
        positionServo(Pickup_ref.position_servo_ref_extend,&hDJI[Motor_Extend_id]);
        positionServo(Pickup_ref.position_servo_ref_overturn,&hDJI[Motor_Overturn_id]);
        xSemaphoreGive(Pickup_ref.xMutex_servo_pickup);

        CanTransmit_DJI_1234(&hcan1,
                             hDJI[0].speedPID.output,
                             hDJI[1].speedPID.output,
                             hDJI[2].speedPID.output,
                             hDJI[3].speedPID.output);

        CanTransmit_DJI_5678(&hcan1,
                             hDJI[4].speedPID.output,
                             hDJI[5].speedPID.output,
                             hDJI[6].speedPID.output,
                             hDJI[7].speedPID.output);
        osDelayUntil(&PreviousWakeTime, 5);
    }
}

void ServoTaskStart()
{
    osThreadDef(servo, ServoTask, osPriorityBelowNormal, 0, 512);
    osThreadCreate(osThread(servo), NULL);
}

void ServoInit()
{
    CANFilterInit(&hcan1);
    hDJI[0].motorType = M3508;
    hDJI[1].motorType = M3508;
    hDJI[2].motorType = M3508;
    hDJI[3].motorType = M3508;
    hDJI[4].motorType = M3508; // 丝杠伸缩
    hDJI[5].motorType = M3508; // 翻滚
    hDJI[6].motorType = M2006; // 爪子

    DJI_Init(); // 大疆电机初始化
}