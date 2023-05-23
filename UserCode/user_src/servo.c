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
#include "state_manage.h"
#include "wtr_vesc.h"

SERVO_REF_FIRE temp_fire_ref;

void ServoTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    for (;;) {
         
        // 推环、递环、Pitch、Yaw轴电机的伺服
        xSemaphoreTake(Fire_ref.xMutex_servo_fire, (TickType_t)10);
        // 射环两个电机伺服拷贝
        temp_fire_ref = ReadServoRefFire(&Fire_ref);
        positionServo(Fire_ref.position_servo_ref_push,&hDJI[Motor_Push_id]);
        positionServo(Fire_ref.position_servo_ref_pass,&hDJI[Motor_pass_id]);
        positionServo(Fire_ref.position_servo_ref_pitch,&hDJI[Motor_Pitch_id]);
        positionServo(Fire_ref.position_servo_ref_yaw,&hDJI[Motor_Yaw_id]);
        xSemaphoreGive(Fire_ref.xMutex_servo_fire);

        VESC_CAN_SET_ERPM(&hvesc[0],temp_fire_ref.speed_servo_ref_left);
        VESC_CAN_SET_ERPM(&hvesc[1],temp_fire_ref.speed_servo_ref_right);

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

void ServoTaskStart(mavlink_controller_t *controldata)
{
    osThreadDef(servo, ServoTask, osPriorityBelowNormal, 0, 512);
    osThreadCreate(osThread(servo), NULL);
}
