/*
 * @Author: szf
 * @Date: 2023-04-02 00:49:59
 * @LastEditTime: 2023-04-02 14:16:52
 * @LastEditors: szf
 * @Description: 射环线程
 * @FilePath: \ER-Upper-Stucture\UserCode\user_src\fire.c
 * @WeChat:szf13373959031
 */
#include "fire.h"
#include "tim.h"

void FireTask(void const *argument)
{
    for(;;)
    {
        osDelay(20);
    } 
}

void FireTestTask(void const *argument)
{
    FireInit();
    osDelay(20);
    
    for(;;)
    {
        // switch(Raw_Data.left)
        // {
        //     case 1:
        //         __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1000);
        //         __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1000); //
        //         break;
        //     case 3:
        //         __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1000);
        //         __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1000); //
        //         break;
        //     case 2:
        //         __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1000);
        //         __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1000); //
        //         break;
        //     default:
        //         break;
        // }
                __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1100);
                __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1100); //
        // 射环机构
        switch(Raw_Data.right)
        {
            case 1:
                positionServo(0, &hDJI[2]);
                positionServo(0, &hDJI[3]);
                break;
            case 3:
                positionServo(20, &hDJI[2]);
                speedServo(200, &hDJI[3]);
                if(hDJI[3].posPID.fdb > 360)
                {
                    positionServo(20, &hDJI[2]);
                    speedServo(0, &hDJI[3]);
                }
                break;
            case 2:
                positionServo(20, &hDJI[2]);
                speedServo(-200, &hDJI[3]);
                if(hDJI[3].posPID.fdb < 0)
                {
                    positionServo(20, &hDJI[2]);
                    speedServo(0, &hDJI[3]);
                }
                break;
            default:
                break;
        }
        
        CanTransmit_DJI_1234(&hcan1,
        hDJI[0].speedPID.output,
        hDJI[1].speedPID.output,
        hDJI[2].speedPID.output,
        hDJI[3].speedPID.output);
        osDelay(1);
    }
}

void FireTaskStart(mavlink_controller_t *controldata)
{
    // osThreadDef(fire, FireTask, osPriorityBelowNormal, 0, 512);
    // osThreadCreate(osThread(fire), controldata);

    osThreadDef(fire_test, FireTestTask, osPriorityBelowNormal, 0, 512);
    osThreadCreate(osThread(fire_test), NULL);
}

void FireInit()
{
    	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
        
        __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,2000); //
        HAL_Delay(2000);
        __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1000); //
        HAL_Delay(2000);

        __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,2000); //
        HAL_Delay(2000);
        __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1000); //
        HAL_Delay(2000);

        CANFilterInit(&hcan1);
        hDJI[2].motorType = M3508; // 丝杠伸缩
        hDJI[3].motorType = M2006; // 翻滚
        DJI_Init(); // 大疆电机初始化
}