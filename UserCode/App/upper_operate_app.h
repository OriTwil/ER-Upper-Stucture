#ifndef __STATE_MANAGE_H__
#define __STATE_MANAGE_H__
#include "upper_common.h"

void StateManagemantTaskStart();
void UpperStateInit();
void PickupSwitchState(PICKUP_STATE target_pick_up_state, UPPER_STATE *current_upper_state);
void PickupSwitchStep(PICKUP_STEP target_pick_up_step, UPPER_STATE *current_upper_state);
void PickupSwitchRing(PICKUP_RING target_pick_up_Ring, UPPER_STATE *current_upper_state);
void FireSwitchNumber(FIRE_NUMBER target_fire_number, UPPER_STATE *current_upper_state);
void SetServoRefPickup(float target_overturn, float target_extend, float target_claw, SERVO_REF_PICKUP *current_pickup_ref);
void SetServoRefPass(float Ref_Pass, SERVO_REF_FIRE *current_fire_ref);
void SetServoRefPush(float ref_push, SERVO_REF_FIRE *current_fire_ref);
void SetFirePwmCcr(int pwm_ccr_left, int pwm_ccr_right, SERVO_REF_FIRE *current_fire_ref);
void SetServoRefFire(float ref_left, float ref_right, SERVO_REF_FIRE *current_fire_ref);
SERVO_REF_FIRE ReadServoRefFire(SERVO_REF_FIRE *current_fire_ref);
void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle);
void SetServoRefFireTrajectory(float ref_pitch, float ref_yaw, SERVO_REF_FIRE *current_fire_ref);
void SetServoRefFireAndPassTrajectory(float ref_pitch, float ref_yaw, float ref_pass, SERVO_REF_FIRE *current_fire_ref);
void SetServoRefOverturnTrajectory(float ref_overturn, SERVO_REF_PICKUP *current_pickup_ref);
void SetFireServoLimitRef(float limit_control, SERVO_REF_FIRE *current_fire_ref);
void SetFireRefServoXC5500(float speed, SERVO_REF_FIRE *current_fire_ref);
void Joystick_Control();

extern UPPER_STATE Upper_state;
extern SERVO_REF_PICKUP Pickup_ref;
extern SERVO_REF_FIRE Fire_ref;
extern VESC_t hvesc[2];
extern mavlink_chassis_to_upper_t Msg_chassis_to_upper;
extern mavlink_joystick_air_t Msg_joystick_air;

#endif