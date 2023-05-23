#ifndef __STATE_MANAGE_H__
#define __STATE_MANAGE_H__
#include "pick_up.h"
#include "user_calculate.h"
#include "semphr.h"
#include "wtr_vesc.h"

typedef enum {
    Overturn,
    Extend,
    Claw_extend,
    Retract,
    Overturn_back,
    Claw_retract
} PICKUP_STEP;

typedef enum {
    Ready,
    Pickup,
    Fire
} PICKUP_STATE;

typedef enum {
    First_Ring = 1,
    Second_Ring,
    Third_Ring,
    Fourth_Ring,
    Fifth_Ring,
    Sixth_Ring,
    Seventh_Ring,
    Eighth_Ring,
    Ninth_Ring,
    Tenth_Ring
} PICKUP_RING;

typedef enum {
    First_Target = 1,
    Second_Target,
    Third_Target,
    Fourth_Target,
    Fifth_Target,
    Sixth_Target,
    Seven_Target,
    Eight_Target
} FIRE_NUMBER;

typedef __IO struct
{
    PICKUP_STEP Pickup_step;
    PICKUP_STATE Pickup_state;
    PICKUP_RING Pickup_ring;
    FIRE_NUMBER Fire_number;
    SemaphoreHandle_t xMutex_upper;
} UPPER_STATE;

typedef __IO struct
{
    float position_servo_ref_overturn;
    float position_servo_ref_extend;
    float position_servo_ref_claw;
    SemaphoreHandle_t xMutex_servo_pickup;
} SERVO_REF_PICKUP;

typedef __IO struct
{
    float speed_servo_ref_left;
    float speed_servo_ref_right;
    int pwm_ccr_left;
    int pwm_ccr_right;
    float position_servo_ref_push;
    float position_servo_ref_pass;
    float position_servo_ref_pitch;
    float position_servo_ref_yaw;
    SemaphoreHandle_t xMutex_servo_fire;
} SERVO_REF_FIRE;

void StateManagemantTaskStart();
void UpperStateInit();
void PickupSwitchState(PICKUP_STATE target_pick_up_state, UPPER_STATE *current_upper_state);
void PickupSwitchStep(PICKUP_STEP target_pick_up_step, UPPER_STATE *current_upper_state);
void PickupSwitchRing(PICKUP_RING target_pick_up_Ring, UPPER_STATE *current_upper_state);
void FireSwitchNumber(FIRE_NUMBER target_fire_number, UPPER_STATE *current_upper_state);
void SetServoRefPickup(float target_overturn, float target_extend, float target_claw, SERVO_REF_PICKUP *current_pickup_ref);
void SetFirePwmCcr(int pwm_ccr_left, int pwm_ccr_right, SERVO_REF_FIRE *current_fire_ref);
void SetServoRefFire(float ref_left, float ref_right, SERVO_REF_FIRE *current_fire_ref);
SERVO_REF_FIRE ReadServoRefFire(SERVO_REF_FIRE *current_fire_ref);
void SetServoRefPush(float ref_push, SERVO_REF_FIRE *current_fire_ref);
void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle);
void SetServoRefFireTrajectory(float ref_pitch, float ref_yaw, SERVO_REF_FIRE *current_fire_ref);
void SetServoRefOverturnTrajectory(float ref_overturn, SERVO_REF_PICKUP *current_pickup_ref);

extern UPPER_STATE Upper_state;
extern SERVO_REF_PICKUP Pickup_ref;
extern SERVO_REF_FIRE Fire_ref;
extern VESC_t hvesc[2];

#endif