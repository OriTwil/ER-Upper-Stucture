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

        // 推环、递环、Pitch、Yaw轴电机的伺服
        // 射环两个电机伺服拷贝
        temp_fire_ref = ReadServoRefFire(&Fire_ref);
        positionServo(temp_fire_ref.position_servo_ref_push, &hDJI[Motor_Push_id]);
        positionServo(temp_fire_ref.position_servo_ref_pass, &hDJI[Motor_pass_id]);
        positionServo(temp_fire_ref.position_servo_ref_pitch, &hDJI[Motor_Pitch_id]);
        positionServo(temp_fire_ref.position_servo_ref_yaw, &hDJI[Motor_Yaw_id]);
        // __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, (int)(temp_fire_ref.speed_servo_ref_left + temp_fire_ref.speed_servo_ref_left_limit));
        // __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, (int)(temp_fire_ref.speed_servo_ref_right + temp_fire_ref.speed_servo_ref_right_limit));

        xSemaphoreTake(Pickup_ref.xMutex_servo_pickup, (TickType_t)10);
        positionServo(Pickup_ref.position_servo_ref_claw, &hDJI[Motor_Claw_id]);
        positionServo(Pickup_ref.position_servo_ref_extend, &hDJI[Motor_Extend_id]);
        positionServo(Pickup_ref.position_servo_ref_overturn, &hDJI[Motor_Overturn_id]);
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
        vTaskDelayUntil(&PreviousWakeTime, 2);
    }
}

void ServoTaskStart()
{
    osThreadDef(servo, ServoTask, osPriorityNormal, 0, 1024);
    osThreadCreate(osThread(servo), NULL);
}

void ServoInit()
{
    CANFilterInit(&hcan1);
    hDJI[Motor_Yaw_id].motorType      = M3508; // 偏航
    hDJI[Motor_Pitch_id].motorType    = M3508; // 俯仰
    hDJI[Motor_Push_id].motorType     = M3508; // 拉环发射
    hDJI[Motor_Extend_id].motorType   = M3508; // 丝杠伸缩
    hDJI[Motor_Overturn_id].motorType = M3508; // 翻滚
    hDJI[Motor_Claw_id].motorType     = M2006; // 爪子
    hDJI[Motor_pass_id].motorType     = M3508; // 递环

    DJI_Init(); // 大疆电机初始化
}