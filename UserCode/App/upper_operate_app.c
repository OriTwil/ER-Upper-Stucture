#include "upper_operate_app.h"
#include "user_config.h"
#include "tim.h"
#include "chassis_remote_control.h"
#include "wtr_uart.h"

UPPER_STATE Upper_state;
SERVO_REF_PICKUP Pickup_ref;
SERVO_REF_FIRE Fire_ref;
VESC_t hvesc[2];
mavlink_chassis_to_upper_t Msg_chassis_to_upper;
mavlink_joystick_air_t Msg_joystick_air;
/**
 * @brief 状态调整线程，根据遥控器、传感器等设计控制逻辑
 * @todo 设计操作手操作逻辑
 * @param (void const *) argument
 * @return
 */
void StateManagemantTask(void const *argument)
{
    // HAL_TIM_PWM_Start(&htim_fire, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim_fire, TIM_CHANNEL_2);

    // __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 2000);
    // __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 2000);
    // vTaskDelay(2000);
    // __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1000);
    // __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1000);
    // vTaskDelay(2000);
    // __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_1, 1175);
    // __HAL_TIM_SetCompare(&htim_fire, TIM_CHANNEL_2, 1175);

    vTaskDelay(20);

    for (;;) {
        vPortEnterCritical();
        Remote_t Raw_Data_temp = Raw_Data;
        vPortExitCritical();

        if (Raw_Data_temp.left == 1) {
            SetServoRefFireTrajectory(0, Fire_Yaw_Initial, &Fire_ref);
        } else if (Raw_Data_temp.left == 3) {
            SetServoRefFireTrajectory(0, Fire_Yaw_Middle, &Fire_ref);
            SetServoRefPush(Fire_Push_Initial, &Fire_ref);
        } else if (Raw_Data_temp.left == 2) {
            SetServoRefFireTrajectory(0, Fire_Yaw_Middle, &Fire_ref);
            SetServoRefPush(Fire_Push, &Fire_ref);
        }

        if (Raw_Data_temp.right == 1) {
            PickupSwitchState(Pickup, &Upper_state);
            PickupSwitchStep(Extend, &Upper_state);
        }

        // if (Raw_Data_temp.right == 1) {
        //     SetServoRefPush(Fire_Push_Initial,&Fire_ref);
        // }
        // if (Raw_Data_temp.right == 3) {
        //     SetServoRefPush(Fire_Push,&Fire_ref);
        // }
    }
}

/**
 * @brief 创建状态调整线程
 *
 * @param (void)
 * @return
 */
void StateManagemantTaskStart()
{

    osThreadDef(statemanagement, StateManagemantTask, osPriorityNormal, 0, 1024);
    osThreadCreate(osThread(statemanagement), NULL);
}

/**
 * @brief 初始化函数
 *
 * @param (void)
 * @bug Motor好像没什么用
 */
void UpperStateInit()
{
    // 互斥锁
    Upper_state.xMutex_upper       = xSemaphoreCreateMutex();
    Pickup_ref.xMutex_servo_pickup = xSemaphoreCreateMutex();
    Fire_ref.xMutex_servo_fire     = xSemaphoreCreateMutex();

    // 上层机构整体状态
    Upper_state.Pickup_state = Ready;
    Upper_state.Pickup_step  = Extend;
    Upper_state.Pickup_ring  = First_Ring;
    Upper_state.Fire_number  = First_Target;

    // 取环组件的伺服值
    Pickup_ref.position_servo_ref_overturn = 0;
    Pickup_ref.position_servo_ref_extend   = 0;
    Pickup_ref.position_servo_ref_claw     = 0;

    // 射环组件的伺服值
    Fire_ref.speed_servo_ref_left     = 0;
    Fire_ref.speed_servo_ref_right    = 0;
    Fire_ref.pwm_ccr_left             = 0;
    Fire_ref.pwm_ccr_right            = 0;
    Fire_ref.position_servo_ref_push  = 0;
    Fire_ref.position_servo_ref_pass  = 0;
    Fire_ref.position_servo_ref_pitch = 0;
    Fire_ref.position_servo_ref_yaw   = 0;

    // VESC
    hvesc[0].controller_id = VESC_Left_id;
    hvesc[0].hcann         = &hcan1;
    hvesc[1].controller_id = VESC_Right_id;
    hvesc[1].hcann         = &hcan1;
}

/**
 * @brief 切换取环状态(	Ready,Pickup,Fire )
 *
 * @param (PICKUP_STATE) target_pick_up_state
 * @param (UPPER_STATE *) current_upper_state
 * @return
 */
void PickupSwitchState(PICKUP_STATE target_pick_up_state, UPPER_STATE *current_upper_state)
{
    xSemaphoreTake(current_upper_state->xMutex_upper, (TickType_t)10);
    current_upper_state->Pickup_state = target_pick_up_state;
    xSemaphoreGive(current_upper_state->xMutex_upper);
}

/**
 * @brief 切换取环所在步骤(	Overturn,Clamp,Overturn_back,Release )
 *
 */
void PickupSwitchStep(PICKUP_STEP target_pick_up_step, UPPER_STATE *current_upper_state)
{
    xSemaphoreTake(current_upper_state->xMutex_upper, (TickType_t)10);
    current_upper_state->Pickup_step = target_pick_up_step;
    xSemaphoreGive(current_upper_state->xMutex_upper);
}

/**
 * @brief 切换取环目标(1 2 3 4 5)
 *
 */
void PickupSwitchRing(PICKUP_RING target_pick_up_Ring, UPPER_STATE *current_upper_state)
{
    xSemaphoreTake(current_upper_state->xMutex_upper, (TickType_t)10);
    current_upper_state->Pickup_ring = target_pick_up_Ring;
    xSemaphoreGive(current_upper_state->xMutex_upper);
}

/**
 * @brief 切换射环的目标
 *
 */
void FireSwitchNumber(FIRE_NUMBER target_fire_number, UPPER_STATE *current_upper_state)
{
    xSemaphoreTake(current_upper_state->xMutex_upper, (TickType_t)10);
    current_upper_state->Fire_number = target_fire_number;
    xSemaphoreGive(current_upper_state->xMutex_upper);
}

/**
 * @brief 设置取环组件各电机伺服值
 *
 */
void SetServoRefPickup(float target_overturn, float target_extend, float target_claw, SERVO_REF_PICKUP *current_pickup_ref)
{
    xSemaphoreTake(current_pickup_ref->xMutex_servo_pickup, (TickType_t)10);
    current_pickup_ref->position_servo_ref_claw     = target_claw;
    current_pickup_ref->position_servo_ref_extend   = target_extend;
    current_pickup_ref->position_servo_ref_overturn = target_overturn;
    xSemaphoreGive(current_pickup_ref->xMutex_servo_pickup);
}

void SetFirePwmCcr(int pwm_ccr_left, int pwm_ccr_right, SERVO_REF_FIRE *current_fire_ref)
{
    xSemaphoreTake(current_fire_ref->xMutex_servo_fire, (TickType_t)10);
    current_fire_ref->pwm_ccr_left  = pwm_ccr_left;
    current_fire_ref->pwm_ccr_right = pwm_ccr_right;
    xSemaphoreGive(current_fire_ref->xMutex_servo_fire);
}

void SetServoRefFire(float ref_left, float ref_right, SERVO_REF_FIRE *current_fire_ref)
{
    xSemaphoreTake(current_fire_ref->xMutex_servo_fire, (TickType_t)10);
    current_fire_ref->speed_servo_ref_left  = ref_left;
    current_fire_ref->speed_servo_ref_right = ref_right;
    xSemaphoreGive(current_fire_ref->xMutex_servo_fire);
}

SERVO_REF_FIRE ReadServoRefFire(SERVO_REF_FIRE *current_fire_ref)
{
    SERVO_REF_FIRE temp_fire_ref;
    xSemaphoreTake(current_fire_ref->xMutex_servo_fire, (TickType_t)10);
    temp_fire_ref = *current_fire_ref;
    xSemaphoreGive(current_fire_ref->xMutex_servo_fire);
    return temp_fire_ref;
}
/**
 * @brief 推环
 *
 */
void SetServoRefPush(float ref_push, SERVO_REF_FIRE *current_fire_ref)
{
    xSemaphoreTake(current_fire_ref->xMutex_servo_fire, (TickType_t)10);
    current_fire_ref->position_servo_ref_push = ref_push;
    xSemaphoreGive(current_fire_ref->xMutex_servo_fire);
}

/**
 * @brief T型速度规划函数
 * @param initialAngle 初始角度
 * @param maxAngularVelocity 最大角速度
 * @param AngularAcceleration 角加速度
 * @param targetAngle 目标角度
 * @param currentTime 当前时间
 * @param currentTime 当前角度
 * @todo 转换为国际单位制
 */
void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle)
{

    float angleDifference = targetAngle - initialAngle;     // 计算到目标位置的角度差
    float sign            = (angleDifference > 0) ? 1 : -1; // 判断角度差的正负(方向)

    float accelerationTime = maxAngularVelocity / AngularAcceleration;                                                      // 加速(减速)总时间
    float constTime        = (fabs(angleDifference) - AngularAcceleration * pow(accelerationTime, 2)) / maxAngularVelocity; // 匀速总时间
    float totalTime        = constTime + accelerationTime * 2;                                                              // 计算到达目标位置所需的总时间

    // 判断能否达到最大速度
    if (constTime > 0) {
        // 根据当前时间判断处于哪个阶段
        if (currentTime <= accelerationTime) {
            // 加速阶段
            *currentAngle = initialAngle + sign * 0.5 * AngularAcceleration * pow(currentTime, 2);
        } else if (currentTime <= accelerationTime + constTime) {
            // 匀速阶段
            *currentAngle = initialAngle + sign * maxAngularVelocity * (currentTime - accelerationTime) + 0.5 * sign * AngularAcceleration * pow(accelerationTime, 2);
        } else if (currentTime <= totalTime) {
            // 减速阶段
            float decelerationTime = currentTime - accelerationTime - constTime;
            *currentAngle          = initialAngle + sign * maxAngularVelocity * constTime + 0.5 * sign * AngularAcceleration * pow(accelerationTime, 2) + sign * (maxAngularVelocity * decelerationTime - 0.5 * AngularAcceleration * pow(decelerationTime, 2));
        } else {
            // 达到目标位置
            *currentAngle = targetAngle;
        }
    } else {
        maxAngularVelocity = sqrt(fabs(angleDifference) * AngularAcceleration);
        accelerationTime   = maxAngularVelocity / AngularAcceleration;
        totalTime          = 2 * accelerationTime;
        constTime          = 0;
        // 根据当前时间判断处于哪个阶段
        if (currentTime <= accelerationTime) {
            // 加速阶段
            *currentAngle = initialAngle + sign * 0.5 * AngularAcceleration * pow(currentTime, 2);
        } else if (currentTime <= totalTime) {
            // 减速阶段
            float decelerationTime = currentTime - accelerationTime; // 减速时间
            *currentAngle          = initialAngle + sign * 0.5 * AngularAcceleration * pow(accelerationTime, 2) + sign * (maxAngularVelocity * decelerationTime - 0.5 * AngularAcceleration * pow(decelerationTime, 2));
        } else {
            // 达到目标位置
            *currentAngle = targetAngle;
        }
    }
}

void SetServoRefFireTrajectory(float ref_pitch, float ref_yaw, SERVO_REF_FIRE *current_fire_ref)
{

    float initialAnglePitch = current_fire_ref->position_servo_ref_pitch;
    float initialAngleYaw   = current_fire_ref->position_servo_ref_yaw; // 电机初始位置

    bool isArrive          = false; // 标志是否达到目标位置
    double differencePitch = 0;
    double differenceYaw   = 0;                   // 和目标之间的角度差
    TickType_t startTime   = xTaskGetTickCount(); // 初始时间

    do {
        TickType_t endTime     = xTaskGetTickCount();
        TickType_t elapsedTime = endTime - startTime;
        float timeSec          = (elapsedTime / (1000.0)); // 获取当前时间/s

        xSemaphoreTake(current_fire_ref->xMutex_servo_fire, (TickType_t)10);
        // 速度规划
        VelocityPlanning(initialAnglePitch, MaxAngularVelocity_Pitch, MotorAngularAcceleration, ref_pitch, timeSec, &(current_fire_ref->position_servo_ref_pitch));
        VelocityPlanning(initialAngleYaw, MaxAngularVelocity_Yaw, MotorAngularAcceleration, ref_yaw, timeSec, &(current_fire_ref->position_servo_ref_yaw));
        xSemaphoreGive(current_fire_ref->xMutex_servo_fire);

        // 判断是否到达目标位置
        differencePitch = fabs(current_fire_ref->position_servo_ref_pitch - ref_pitch);
        differenceYaw   = fabs(current_fire_ref->position_servo_ref_yaw - ref_yaw);
        if (differencePitch < 0.1 && differenceYaw < 0.1) {
            isArrive = true;
        }

        vTaskDelay(2);
    } while (!isArrive);
}

void SetServoRefOverturnTrajectory(float ref_overturn, SERVO_REF_PICKUP *current_pickup_ref)
{
    float initialAngleOverturn = current_pickup_ref->position_servo_ref_overturn;

    bool isArrive             = false; // 标志是否达到目标位置
    double differenceOverturn = 0;
    TickType_t startTime      = xTaskGetTickCount(); // 初始时间

    do {
        TickType_t endTime     = xTaskGetTickCount();
        TickType_t elapsedTime = endTime - startTime;
        float timeSec          = (elapsedTime / (1000.0)); // 获取当前时间/s

        xSemaphoreTake(current_pickup_ref->xMutex_servo_pickup, (TickType_t)10);
        // 速度规划
        VelocityPlanning(initialAngleOverturn, MaxAngularVelocity_Overturn, MotorAngularAcceleration, ref_overturn, timeSec, &(current_pickup_ref->position_servo_ref_overturn));
        xSemaphoreGive(current_pickup_ref->xMutex_servo_pickup);

        // 判断是否到达目标位置
        differenceOverturn = fabs(current_pickup_ref->position_servo_ref_overturn - ref_overturn);
        if (differenceOverturn < 0.1) {
            isArrive = true;
        }
        vTaskDelay(2);
    } while (!isArrive);
}

void Joystick_Control()
{
    if (ReadJoystickButtons(Msg_joystick_air, Btn_RightCrossUp)) {
        PickupSwitchState(Pickup, &Upper_state);
    }
    // Yaw转向
    if (ReadJoystickButtons(Msg_joystick_air, Btn_LeftCrossDown)) {
        SetServoRefFireTrajectory(0, Fire_Yaw_Middle, &Fire_ref);
    }
    if (ReadJoystickButtons(Msg_joystick_air, Btn_RightCrossDown)) {
        SetServoRefFireTrajectory(0, Fire_Yaw_Initial, &Fire_ref);
    }
    // Push
    if (ReadJoystickButtons(Msg_joystick_air, Btn_LeftCrossUp)) {
        SetServoRefPush(Fire_Push, &Fire_ref);
        vTaskDelay(3000);
        SetServoRefPush(Fire_Push_Initial, &Fire_ref);
    }
}