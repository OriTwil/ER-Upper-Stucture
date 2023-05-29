/*
 * @Author: szf
 * @Date: 2023-03-08 19:12:32
 * @LastEditTime: 2023-04-02 14:41:33
 * @LastEditors: szf
 * @Description:
 * @FilePath: \ER-Upper-Stucture\UserCode\user_src\pick_up.c
 * @WeChat:szf13373959031
 */
#include "pick_up.h"
#include "wtr_uart.h"
#include "state_manage.h"
#include "tim.h"
Button button = {
    .button_min_time = 500,
    .last_tick       = 0,
};

void PickUpTask(void const *argument)
{
    osDelay(200);

    for (;;) {
        switch (Upper_state.Pickup_state) {
            case Ready:

                break;
            case Pickup:
                SetServoRefFire(0, 0, &Pickup_ref);
				SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Initial, ClawAngle_Initial, &Pickup_ref);
                // 取环
                switch (Upper_state.Pickup_step) {
                    case Overturn:
                        SetServoRefOverturnTrajectory(OverturnAngle, &Pickup_ref);
                        if (hDJI[Motor_Overturn_id].posPID.fdb < OverturnAngle) {
                            PickupSwitchStep(Extend, &Pickup_ref);
                        }
                        break;
                    case Extend:
                        SetServoRefPickup(OverturnAngle, ExtendAngle, ClawAngle_Initial, &Pickup_ref);
                        if (hDJI[Motor_Extend_id].posPID.fdb > ExtendAngle) {
                            PickupSwitchStep(Claw_extend, &Pickup_ref);
                        }
                        break;
                    case Claw_extend:
                        SetServoRefPickup(OverturnAngle, ExtendAngle, ClawAngle, &Pickup_ref);
                        if (hDJI[Motor_Claw_id].posPID.fdb > ClawAngle) {
                            PickupSwitchStep(Retract, &Pickup_ref);
                        }
                        break;
                    case Retract:
                        SetServoRefPickup(OverturnAngle, ExtendAngle_10, ClawAngle, &Pickup_ref);
                        if (hDJI[Motor_Extend_id].posPID.fdb < ExtendAngle_10) {
                            PickupSwitchStep(Overturn_back, &Pickup_ref);
                        }
                        break;
                    case Overturn_back:
                        SetServoRefOverturnTrajectory(ExtendAngle_10, &Pickup_ref);
                        if (hDJI[Motor_Overturn_id].posPID.fdb > OverturnAngle_Initial) {
                            PickupSwitchStep(Claw_retract, &Pickup_ref);
                        }
                        break;
                    case Claw_retract:
                        SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_10, ClawAngle_Initial, &Pickup_ref);
                        if (hDJI[Motor_Claw_id].posPID.fdb < ClawAngle_Initial) {
                            PickupSwitchState(Ready, &Upper_state);
                        }
                        break;
                    default:
                        break;
                }
                break;
            case Fire:
				switch(Upper_state.Fire_number)
				{
					case First_Target:
					SetServoRefPickup(OverturnAngle_Initial,ExtendAngle_10,ClawAngle_Initial,&Pickup_ref);
					break;
					case Second_Target:
					break;
					case Third_Target:
					break;
					case Fourth_Target:
					break;
					case Fifth_Target:
					break;
					case Sixth_Target:
					break;
					case Seven_Target:
					break;
					case Eight_Target:
					break;
				}
                break;
        }

        CanTransmit_DJI_5678(&hcan1,
                             hDJI[4].speedPID.output,
                             hDJI[5].speedPID.output,
                             hDJI[6].speedPID.output,
                             hDJI[7].speedPID.output);
        osDelay(1);
    }
}

void PickUpTestTask(void const *argument)
{
    PickUpInit(); // 取环机构初始化
    osDelay(20);
    for (;;) {
        vTaskDelay(20);
    }
}

void PickUpTaskStart(mavlink_controller_t *controldata)
{
    // osThreadDef(pickup, PickUpTask, osPriorityBelowNormal, 0, 512);
    // osThreadCreate(osThread(pickup), controldata);

    osThreadDef(upper_test, PickUpTestTask, osPriorityBelowNormal, 0, 512);
    osThreadCreate(osThread(upper_test), NULL);
}

void DeadBand(double x, double y, double *new_x, double *new_y, double threshould)
{
    double length     = sqrt(x * x + y * y);
    double new_length = length - threshould;

    if (new_length <= 0) {
        *new_x = 0;
        *new_y = 0;
        return;
    }

    double k = new_length / length;

    *new_x = x * k;
    *new_y = y * k;
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