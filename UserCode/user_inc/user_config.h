/*** 
 * @Author: Xieyang
 * @Date: 2023-02-23 18:51:00
 * @LastEditTime: 2023-02-23 18:51:13
 * @LastEditors: szf
 * @Description: 
 * @FilePath: \ER\Usercode\user_inc\useful_constant.h
 * @@WeChat:szf13373959031
 */
/**
 * @brief 一些有用的常量，基本上是从 GCC 的 math.h 中复制下来的(因为 ARMCC 的库里貌似没有)
 * 
 */

#pragma once

/* Natural log of 2 */
#define _M_LN2        0.693147180559945309417

#define MAXFLOAT	3.40282347e+38F

#define M_E		2.7182818284590452354
#define M_LOG2E		1.4426950408889634074
#define M_LOG10E	0.43429448190325182765
#define M_LN2		_M_LN2
#define M_LN10		2.30258509299404568402
#define M_PI		3.14159265358979323846

#ifndef M_PI_2
#define M_PI_2		1.57079632679489661923
#endif // !M_PI_2

#define M_PI_4		0.78539816339744830962
#define M_1_PI		0.31830988618379067154
#define M_2_PI		0.63661977236758134308
#define M_2_SQRTPI	1.12837916709551257390
#define M_SQRT2		1.41421356237309504880
#define M_SQRT1_2	0.70710678118654752440

#define M_TWOPI         (M_PI * 2.0)
#define M_3PI_4		2.3561944901923448370E0
#define M_SQRTPI        1.77245385090551602792981
#define M_LN2LO         1.9082149292705877000E-10
#define M_LN2HI         6.9314718036912381649E-1
#define M_SQRT3	1.73205080756887719000
#define M_IVLN10        0.43429448190325182765 /* 1 / log(10) */
#define M_LOG2_E        _M_LN2
#define M_INVLN2        1.4426950408889633870E0  /* 1 / log(2) */

/**
 * @brief id
 * 
 */
#define VESC_Left_id 0x01
#define VESC_Right_id 0x02


#define Motor_Pitch_id 2
#define Motor_Yaw_id 1
#define Motor_Push_id 0
#define Motor_pass_id 3 
#define Motor_Overturn_id 4
#define Motor_Extend_id 5
#define Motor_Claw_id 6

/**
 * @brief 参数
 * 
 */
// 电机的最大角速度 角加速度
#define MaxAngularVelocity_Pitch 10
#define MaxAngularVelocity_Yaw   10
#define MaxAngularVelocity_Overturn 10

#define MotorAngularAcceleration 10


#define OverturnAngle_Initial 0
#define ExtendAngle_Initial 0 //todo 需要确定每次初始位置都一样
#define ClawAngle_Initial 0

#define OverturnAngle -729
#define ExtendAngle 5200
#define ExtendAngle_10 5000
#define ExtendAngle_9 4800
#define ExtendAngle_8 4600
#define ExtendAngle_7 4400
#define ExtendAngle_6 4200
#define ExtendAngle_5 4000
#define ExtendAngle_4 3800
#define ExtendAngle_3 3600
#define ExtendAngle_2 3400
#define ExtendAngle_1 3200
#define ClawAngle -36

#define ExtendAngle_Retract 2000


