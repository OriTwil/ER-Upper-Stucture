#ifndef __UPPER_COMMON_H__
#define __UPPER_COMMON_H__

#include "upper_pick_up.h"
#include "wtr_calculate.h"
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
    Fire_StepOne,
    Fire_StepTwo,
    Fire_Ready
} PICKUP_STATE;

typedef enum {
    Zero_Ring  = 0,
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
    Zero_Target  = 0,
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
    float position_servo_ref_push; // 拉环发射
    float position_servo_ref_pass; // 打环递环
    float position_servo_ref_pitch;
    float position_servo_ref_yaw;
    SemaphoreHandle_t xMutex_servo_fire;
} SERVO_REF_FIRE;

#endif