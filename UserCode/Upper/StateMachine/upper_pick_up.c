/*
 * @Author: szf
 * @Date: 2023-03-08 19:12:32
 * @LastEditTime: 2023-04-02 14:41:33
 * @LastEditors: szf
 * @Description:
 * @FilePath: \ER-Upper-Stucture\UserCode\user_src\upper_pick_up.c
 * @WeChat:szf13373959031
 */
#include "upper_pick_up.h"
#include "wtr_uart.h"
#include "upper_operate_app.h"
#include "user_config.h"
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
                SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Initial, ClawAngle_Initial, &Pickup_ref);
                break;
            case Pickup:
                // 取环
                switch (Upper_state.Pickup_step) {
                    case Overturn:
                        SetServoRefOverturnTrajectory(OverturnAngle, &Pickup_ref);
                        break;
                    case Extend:
                        SetServoRefPickup(OverturnAngle_Initial, ExtendAngle, ClawAngle_Initial, &Pickup_ref);
                        if (fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle) < 2) {
                            PickupSwitchStep(Claw_extend, &Upper_state);
                        }
                        break;
                    case Claw_extend:
                        SetServoRefPickup(OverturnAngle_Initial, ExtendAngle, ClawAngle, &Pickup_ref);
                        if (fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle) < 2) {
                            PickupSwitchStep(Retract, &Upper_state);
                        }
                        break;
                    case Retract:
                        SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_10, ClawAngle, &Pickup_ref);
                        if (fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle_10) < 2) {
                            PickupSwitchStep(Overturn_back, &Upper_state);
                        }
                        break;
                    case Overturn_back:
                        SetServoRefOverturnTrajectory(OverturnAngle, &Pickup_ref);
                        if (fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle_Initial) < 2) {
                            PickupSwitchStep(Claw_retract, &Upper_state);
                        }
                        break;
                    case Claw_retract:
                        SetServoRefPickup(OverturnAngle, ExtendAngle_10, ClawAngle_Initial, &Pickup_ref);
                        if (fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 2) {
                            PickupSwitchStep(Overturn, &Upper_state);
                        }
                        break;
                    default:
                        break;
                }
                break;
            case Fire:
                // SetServoRefPickup(OverturnAngle, ExtendAngle_10, ClawAngle_Initial, &Pickup_ref);
                // switch(Upper_state.Fire_number)
                // {
                // 	case First_Target:
                // 	break;
                // 	case Second_Target:
                // 	break;
                // 	case Third_Target:
                // 	break;
                // 	case Fourth_Target:
                // 	break;
                // 	case Fifth_Target:
                // 	break;
                // 	case Sixth_Target:
                // 	break;
                // 	case Seven_Target:
                // 	break;
                // 	case Eight_Target:
                // 	break;
                // }
                break;
        }
        vTaskDelay(1);
    }
}

void PickUpTestTask(void const *argument)
{
    osDelay(20);
    for (;;) {
        vTaskDelay(20);
    }
}

void PickUpTaskStart()
{
    osThreadDef(pickup, PickUpTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(pickup), NULL);

    // osThreadDef(upper_test, PickUpTestTask, osPriorityBelowNormal, 0, 512);
    // osThreadCreate(osThread(upper_test), NULL);
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

