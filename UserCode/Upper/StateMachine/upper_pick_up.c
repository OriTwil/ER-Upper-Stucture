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

uint32_t point     = 0;
bool is_Tenth_Ring = false;
void PickUpTask(void const *argument)
{
    vTaskDelay(200);

    for (;;) {
        switch (Upper_state.Pickup_state) {
            case Ready:
                SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Initial, ClawAngle_Initial, &Pickup_ref);
                break;
            case Pickup:
                // 取环
                switch (Upper_state.Pickup_step) {
                    case Overturn:
                        SetServoRefPass(Pickup_Pass, &Fire_ref);
                        if (fabs(hDJI[Motor_pass_id].posPID.fdb - Pickup_Pass) < 5) {
                            PickupSwitchStep(Extend, &Upper_state);
                        }
                        break;
                    case Extend:
                        SetServoRefPickup(OverturnAngle_Initial, ExtendAngle, ClawAngle_Initial, &Pickup_ref);
                        if (fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle_Initial) < 1.0 &&
                            fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle) < 40 &&
                            fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 1.0) {
                            PickupSwitchStep(Claw_extend, &Upper_state);
                        }
                        break;
                    case Claw_extend:
                        SetServoRefPickup(OverturnAngle_Initial, ExtendAngle, ClawAngle, &Pickup_ref);
                        if (fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle_Initial) < 1.0 &&
                            fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle) < 1.0 &&
                            fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle) < 5.0) {
                            PickupSwitchStep(Retract, &Upper_state);
                        }
                        break;
                    case Retract:
                        SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_back, ClawAngle, &Pickup_ref);
                        if (fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle_Initial) < 1.0 &&
                            fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle_back) < 30.0 &&
                            fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle) < 1.0) {
                            PickupSwitchStep(Overturn_back, &Upper_state);
                        }
                        break;
                    case Overturn_back:
                        SetServoRefPass(-5, &Fire_ref);
                        vTaskDelay(100);
                        SetServoRefOverturnTrajectory(OverturnAngle, &Pickup_ref);
                        vTaskDelay(20);
                        // SetServoRefPass(-23, &Fire_ref);
                        SetServoRefPickup(OverturnAngle, 13500, ClawAngle, &Pickup_ref);
                        while (fabs(hDJI[Motor_Extend_id].posPID.fdb - 13500) > 5.0) {
                            vTaskDelay(1);
                        }
                        PickupSwitchStep(Claw_retract, &Upper_state);
                        break;
                    case Claw_retract:
                        SetServoRefPass(-23, &Fire_ref);
                        while (fabs(hDJI[Motor_pass_id].posPID.fdb - (-23)) > 5.0) {
                            vTaskDelay(1);
                        }
                        SetServoRefPickup(OverturnAngle, 13500, ClawAngle_Initial, &Pickup_ref);
                        if (fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle) < 0.5 &&
                            fabs(hDJI[Motor_Extend_id].posPID.fdb - 13500) < 0.5 &&
                            fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 3.0) {
                            SetServoRefFireTrajectory(-600, 0, &Fire_ref);
                            PickupSwitchStep(Overturn, &Upper_state);
                            PickupSwitchState(Fire_Ready, &Upper_state);
                        }
                        break;
                    default:
                        break;
                }
                break;
            case Fire_Ready:
                SetFireRefServoXC5500(1000, &Fire_ref);
                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                SetServoRefPass(Fire_pass_ready, &Fire_ref);
                SetServoRefFireTrajectory(Pitch_Fire_Ready, Yaw_Fire_Ready, &Fire_ref);
                vTaskDelay(10);
                break;
            case Fire_StepOne:
                switch (Upper_state.Pickup_ring) {
                    case First_Ring:
                        SetServoRefPickup(OverturnAngle, ExtendAngle_1, ClawAngle_Initial, &Pickup_ref);
                        while (!(fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle) < 1.0 &&
                                 fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle_1) < 5.0 &&
                                 fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 1.0)) {
                            vTaskDelay(1);
                        }
                        SetServoRefFireTrajectory(Get_Pitch_1, Yaw_Fire_Ready, &Fire_ref);
                        SetServoRefPass(Fire_Pass_1, &Fire_ref);
                        while (hDJI[Motor_pass_id].posPID.fdb - Fire_pass_threshold <= 0) {
                            vTaskDelay(1);
                        }
                        SetServoRefPass(Fire_pass_ready, &Fire_ref);
                        PickupSwitchRing(Second_Ring, &Upper_state);
                        PickupSwitchState(Fire_StepTwo, &Upper_state);
                        break;
                    case Second_Ring:
                        SetServoRefPickup(OverturnAngle, ExtendAngle_2, ClawAngle_Initial, &Pickup_ref);
                        while (!(fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle) < 1.0 &&
                                 fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle_2) < 5.0 &&
                                 fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 1.0)) {
                            vTaskDelay(1);
                        }
                        SetServoRefFireTrajectory(Get_Pitch_2, Yaw_Fire_Ready, &Fire_ref);
                        SetServoRefPass(Fire_Pass_2, &Fire_ref);
                        while (hDJI[Motor_pass_id].posPID.fdb - Fire_pass_threshold <= 0) {
                            vTaskDelay(1);
                        }
                        SetServoRefPass(Fire_pass_ready, &Fire_ref);
                        PickupSwitchRing(Third_Ring, &Upper_state);
                        PickupSwitchState(Fire_StepTwo, &Upper_state);
                        break;
                    case Third_Ring:
                        SetServoRefPickup(OverturnAngle, ExtendAngle_3, ClawAngle_Initial, &Pickup_ref);
                        while (!(fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle) < 1.0 &&
                                 fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle_3) < 5.0 &&
                                 fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 1.0)) {
                            vTaskDelay(1);
                        }
                        SetServoRefFireTrajectory(Get_Pitch_3, Yaw_Fire_Ready, &Fire_ref);
                        SetServoRefPass(Fire_Pass_3, &Fire_ref);
                        while (hDJI[Motor_pass_id].posPID.fdb - Fire_pass_threshold <= 0) {
                            vTaskDelay(1);
                        }
                        SetServoRefPass(Fire_pass_ready, &Fire_ref);
                        PickupSwitchRing(Fourth_Ring, &Upper_state);
                        PickupSwitchState(Fire_StepTwo, &Upper_state);
                        break;
                    case Fourth_Ring:
                        SetServoRefPickup(OverturnAngle, ExtendAngle_4, ClawAngle_Initial, &Pickup_ref);
                        while (!(fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle) < 1.0 &&
                                 fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle_4) < 5.0 &&
                                 fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 1.0)) {
                            vTaskDelay(1);
                        }
                        SetServoRefFireTrajectory(Get_Pitch_4, Yaw_Fire_Ready, &Fire_ref);
                        SetServoRefPass(Fire_Pass_4, &Fire_ref);
                        while (hDJI[Motor_pass_id].posPID.fdb - Fire_pass_threshold <= 0) {
                            vTaskDelay(1);
                        }
                        SetServoRefPass(Fire_pass_ready, &Fire_ref);
                        PickupSwitchRing(Fifth_Ring, &Upper_state);
                        PickupSwitchState(Fire_StepTwo, &Upper_state);
                        break;
                    case Fifth_Ring:
                        SetServoRefPickup(OverturnAngle, ExtendAngle_5, ClawAngle_Initial, &Pickup_ref);
                        while (!(fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle) < 1.0 &&
                                 fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle_5) < 5.0 &&
                                 fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 1.0)) {
                            vTaskDelay(1);
                        }
                        SetServoRefFireTrajectory(Get_Pitch_5, Yaw_Fire_Ready, &Fire_ref);
                        SetServoRefPass(Fire_Pass_5, &Fire_ref);
                        while (hDJI[Motor_pass_id].posPID.fdb - Fire_pass_threshold <= 0) {
                            vTaskDelay(1);
                        }
                        SetServoRefPass(Fire_pass_ready, &Fire_ref);
                        PickupSwitchRing(Sixth_Ring, &Upper_state);
                        PickupSwitchState(Fire_StepTwo, &Upper_state);
                        break;
                    case Sixth_Ring:
                        SetServoRefPickup(OverturnAngle, ExtendAngle_6, ClawAngle_Initial, &Pickup_ref);
                        while (!(fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle) < 1.0 &&
                                 fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle_6) < 5.0 &&
                                 fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 1.0)) {
                            vTaskDelay(1);
                        }
                        SetServoRefFireTrajectory(Get_Pitch_6, Yaw_Fire_Ready, &Fire_ref);
                        SetServoRefPass(Fire_Pass_6, &Fire_ref);
                        while (hDJI[Motor_pass_id].posPID.fdb - Fire_pass_threshold <= 0) {
                            vTaskDelay(1);
                        }
                        SetServoRefPass(Fire_pass_ready, &Fire_ref);
                        PickupSwitchRing(Seventh_Ring, &Upper_state);
                        PickupSwitchState(Fire_StepTwo, &Upper_state);
                        break;
                    case Seventh_Ring:
                        SetServoRefPickup(OverturnAngle, ExtendAngle_7, ClawAngle_Initial, &Pickup_ref);
                        while (!(fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle) < 1.0 &&
                                 fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle_7) < 5.0 &&
                                 fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 1.0)) {
                            vTaskDelay(1);
                        }
                        SetServoRefFireTrajectory(Get_Pitch_7, Yaw_Fire_Ready, &Fire_ref);
                        SetServoRefPass(Fire_Pass_7, &Fire_ref);
                        while (hDJI[Motor_pass_id].posPID.fdb - Fire_pass_threshold <= 0) {
                            vTaskDelay(1);
                        }
                        SetServoRefPass(Fire_pass_ready, &Fire_ref);
                        PickupSwitchRing(Eighth_Ring, &Upper_state);
                        PickupSwitchState(Fire_StepTwo, &Upper_state);
                        break;
                    case Eighth_Ring:
                        SetServoRefPickup(OverturnAngle, ExtendAngle_8, ClawAngle_Initial, &Pickup_ref);
                        while (!(fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle) < 1.0 &&
                                 fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle_8) < 5.0 &&
                                 fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 1.0)) {
                            vTaskDelay(1);
                        }
                        SetServoRefFireTrajectory(Get_Pitch_8, Yaw_Fire_Ready, &Fire_ref);
                        SetServoRefPass(Fire_Pass_8, &Fire_ref);
                        while (hDJI[Motor_pass_id].posPID.fdb - Fire_pass_threshold <= 0) {
                            vTaskDelay(1);
                        }
                        SetServoRefPass(Fire_pass_ready, &Fire_ref);
                        PickupSwitchRing(Ninth_Ring, &Upper_state);
                        PickupSwitchState(Fire_StepTwo, &Upper_state);
                        break;
                    case Ninth_Ring:
                        SetServoRefPickup(OverturnAngle, ExtendAngle_9, ClawAngle_Initial, &Pickup_ref);
                        while (!(fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle) < 1.0 &&
                                 fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle_9) < 5.0 &&
                                 fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 1.0)) {
                            vTaskDelay(1);
                        }
                        SetServoRefFireTrajectory(Get_Pitch_9, Yaw_Fire_Ready, &Fire_ref);
                        SetServoRefPass(Fire_Pass_9, &Fire_ref);
                        while (hDJI[Motor_pass_id].posPID.fdb - Fire_pass_threshold <= 0) {
                            vTaskDelay(1);
                        }
                        SetServoRefPass(Fire_pass_ready, &Fire_ref);
                        PickupSwitchRing(Tenth_Ring, &Upper_state);
                        PickupSwitchState(Fire_StepTwo, &Upper_state);
                        break;
                    case Tenth_Ring:
                        SetServoRefPickup(OverturnAngle, ExtendAngle_10, ClawAngle_Initial, &Pickup_ref);
                        while (!(fabs(hDJI[Motor_Overturn_id].posPID.fdb - OverturnAngle) < 1.0 &&
                                 fabs(hDJI[Motor_Extend_id].posPID.fdb - ExtendAngle_10) < 5.0 &&
                                 fabs(hDJI[Motor_Claw_id].posPID.fdb - ClawAngle_Initial) < 1.0)) {
                            vTaskDelay(1);
                        }
                        SetServoRefFireTrajectory(Get_Pitch_10, Yaw_Fire_Ready, &Fire_ref);
                        SetServoRefPass(Fire_Pass_10, &Fire_ref);
                        while (hDJI[Motor_pass_id].posPID.fdb - Fire_pass_threshold <= 0) {
                            vTaskDelay(1);
                        }
                        SetServoRefPass(Fire_pass_ready, &Fire_ref);
                        PickupSwitchRing(First_Ring, &Upper_state);
                        PickupSwitchState(Fire_StepTwo, &Upper_state);
                        is_Tenth_Ring = true;
                        break;
                    default:
                        break;
                }
                break;
            case Fire_StepTwo:
                switch (point) {
                    case 0:
                        switch (Upper_state.Fire_number) {
                            case First_Target:
                                vTaskDelay(2000);

                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                while (fabs(hDJI[Motor_Push_id].posPID.fdb - Fire_Push_Transition_2) >= 3.0) {
                                    vTaskDelay(1);
                                }
                                SetFireRefServoXC5500(1000, &Fire_ref);
                                vTaskDelay(2000);
                                SetServoRefFireTrajectory(Fire_Pitch_1_1, Yaw_Fire_Ready, &Fire_ref);
                                vTaskDelay(20);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                while (fabs(hDJI[Motor_Push_id].posPID.fdb - Fire_Push) >= 1.0) {
                                    vTaskDelay(1);
                                }
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            default:
                                break;
                        }
                        if (is_Tenth_Ring) {
                            is_Tenth_Ring = false;
                            __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                            __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                            SetServoRefFireTrajectory(Pitch_Back_Transition, Yaw_Fire_Ready, &Fire_ref);
                            SetServoRefFireTrajectory(Pitch_Initial, Yaw_Initial, &Fire_ref);
                            SetServoRefPass(Fire_Pass_Initial, &Fire_ref);
                            SetServoRefPickup(OverturnAngle, ExtendAngle_Transition, ClawAngle_Initial, &Pickup_ref);
                            vTaskDelay(5000);
                            SetServoRefOverturnTrajectory(OverturnAngle_Initial, &Pickup_ref);
                            vTaskDelay(5);
                            SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Initial, ClawAngle_Initial, &Pickup_ref);
                            PickupSwitchStep(Overturn, &Upper_state);
                            PickupSwitchState(Ready, &Upper_state);
                        }
                        break;
                    case 1:
                        switch (Upper_state.Fire_number) {
                            case First_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                while (fabs(hDJI[Motor_Push_id].posPID.fdb - Fire_Push_Transition) >= 3.0) {
                                    vTaskDelay(1);
                                }
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_1_1, Fire_Yaw_1_1, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            case Second_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_1_2, Fire_Yaw_1_2, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            case Third_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_1_3, Fire_Yaw_1_3, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            default:
                                break;
                        }
                        if (is_Tenth_Ring) {
                            is_Tenth_Ring = false;
                            SetServoRefFireTrajectory(Pitch_Initial, Yaw_Initial, &Fire_ref);
                            SetServoRefPickup(OverturnAngle, ExtendAngle_Transition, ClawAngle_Initial, &Pickup_ref);
                            vTaskDelay(5);
                            SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Transition, ClawAngle_Initial, &Pickup_ref);
                            vTaskDelay(5);
                            SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Initial, ClawAngle_Initial, &Pickup_ref);
                            PickupSwitchStep(Overturn, &Upper_state);
                            PickupSwitchState(Ready, &Upper_state);
                        }
                        break;
                    case 2:
                        switch (Upper_state.Fire_number) {
                            case First_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_2_1, Fire_Yaw_2_1, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            case Second_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_2_2, Fire_Yaw_2_2, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            case Third_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_2_3, Fire_Yaw_2_3, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            default:
                                break;
                        }
                        if (is_Tenth_Ring) {
                            is_Tenth_Ring = false;
                            __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                            __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                            SetServoRefFireTrajectory(Pitch_Back_Transition, Yaw_Fire_Ready, &Fire_ref);
                            SetServoRefFireTrajectory(Pitch_Initial, Yaw_Initial, &Fire_ref);
                            SetServoRefPass(Fire_Pass_Initial, &Fire_ref);
                            SetServoRefPickup(OverturnAngle, ExtendAngle_Transition, ClawAngle_Initial, &Pickup_ref);
                            vTaskDelay(3000);
                            SetServoRefOverturnTrajectory(OverturnAngle_Initial, &Pickup_ref);
                            vTaskDelay(5);
                            SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Initial, ClawAngle_Initial, &Pickup_ref);
                            PickupSwitchStep(Overturn, &Upper_state);
                            PickupSwitchState(Ready, &Upper_state);
                        }
                        break;
                    case 3:
                        switch (Upper_state.Fire_number) {
                            case First_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_3_1, Fire_Yaw_3_1, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            case Second_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_3_2, Fire_Yaw_3_2, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            case Third_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_3_3, Fire_Yaw_3_3, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            default:
                                break;
                        }
                        if (is_Tenth_Ring) {
                            is_Tenth_Ring = false;
                            SetServoRefFireTrajectory(Pitch_Initial, Yaw_Initial, &Fire_ref);
                            SetServoRefPickup(OverturnAngle, ExtendAngle_Transition, ClawAngle_Initial, &Pickup_ref);
                            vTaskDelay(5);
                            SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Transition, ClawAngle_Initial, &Pickup_ref);
                            vTaskDelay(5);
                            SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Initial, ClawAngle_Initial, &Pickup_ref);
                            PickupSwitchStep(Overturn, &Upper_state);
                            PickupSwitchState(Ready, &Upper_state);
                        }
                        break;
                    case 4:
                        switch (Upper_state.Fire_number) {
                            case First_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_4_1, Fire_Yaw_4_1, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            case Second_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_4_2, Fire_Yaw_4_2, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            case Third_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_4_3, Fire_Yaw_4_3, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            default:
                                break;
                        }
                        if (is_Tenth_Ring) {
                            is_Tenth_Ring = false;
                            SetServoRefFireTrajectory(Pitch_Initial, Yaw_Initial, &Fire_ref);
                            SetServoRefPickup(OverturnAngle, ExtendAngle_Transition, ClawAngle_Initial, &Pickup_ref);
                            vTaskDelay(5);
                            SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Transition, ClawAngle_Initial, &Pickup_ref);
                            vTaskDelay(5);
                            SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Initial, ClawAngle_Initial, &Pickup_ref);
                            PickupSwitchStep(Overturn, &Upper_state);
                            PickupSwitchState(Ready, &Upper_state);
                        }
                        break;
                    case 5:
                        switch (Upper_state.Fire_number) {
                            case First_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_5_1, Fire_Yaw_5_1, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            case Second_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_5_2, Fire_Yaw_5_2, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            case Third_Target:
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
                                __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Transition, &Fire_ref);
                                vTaskDelay(500);
                                SetServoRefFireTrajectory(-650, Yaw_Fire_Ready, &Fire_ref);
                                SetServoRefPush(Fire_Push_Transition_2, &Fire_ref);
                                SetServoRefFireTrajectory(Fire_Pitch_5_3, Fire_Yaw_5_3, &Fire_ref);
                                vTaskDelay(800);
                                SetServoRefPush(Fire_Push, &Fire_ref);
                                vTaskDelay(1000);
                                SetServoRefPush(Fire_Push_Initial, &Fire_ref);
                                PickupSwitchState(Fire_Ready, &Upper_state);
                                break;
                            default:
                                break;
                        }
                        if (is_Tenth_Ring) {
                            is_Tenth_Ring = false;
                            SetServoRefFireTrajectory(Pitch_Initial, Yaw_Initial, &Fire_ref);
                            SetServoRefPickup(OverturnAngle, ExtendAngle_Transition, ClawAngle_Initial, &Pickup_ref);
                            vTaskDelay(5);
                            SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Transition, ClawAngle_Initial, &Pickup_ref);
                            vTaskDelay(5);
                            SetServoRefPickup(OverturnAngle_Initial, ExtendAngle_Initial, ClawAngle_Initial, &Pickup_ref);
                            PickupSwitchStep(Overturn, &Upper_state);
                            PickupSwitchState(Ready, &Upper_state);
                        }
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
    }
    vTaskDelay(50);
}

void PickUpTestTask(void const *argument)
{
    vTaskDelay(20);
    for (;;) {
        SetServoRefPickup(OverturnAngle_Initial, ExtendAngle, ClawAngle_Initial, &Pickup_ref);
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
// void SetServoRefPassTrajectory(float target_angle, float threshold_angle, SERVO_REF_FIRE *current_fire_ref)
// {
//     xSemaphoreTake(current_fire_ref->xMutex_servo_fire, (TickType_t)10);
//     current_fire_ref->position_servo_ref_pass = target_angle;
//     xSemaphoreGive(current_fire_ref->xMutex_servo_fire);
//     while (hDJI[Motor_pass_id].posPID.fdb - threshold_angle <= 0) {
//         vTaskDelay(1);
//     }
//     xSemaphoreTake(current_fire_ref->xMutex_servo_fire, (TickType_t)10);
//     current_fire_ref->position_servo_ref_pass = target_angle;
//     xSemaphoreGive(current_fire_ref->xMutex_servo_fire);
// }