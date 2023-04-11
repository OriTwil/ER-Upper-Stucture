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
Button button = {
    .button_min_time = 500,
    .last_tick       = 0,
};
bool Pickup_state = Finished;
Mode Pickup_mode = Motionless;

void PickUpTask(void const *argument)
{
    const mavlink_controller_t *ctrl_data = argument;
    osDelay(200);

    for (;;) 
	{
        if (ctrl_data->buttons & (1 << 6)) 
		{
            if (button.last_tick + button.button_min_time < HAL_GetTick()) 
			{
                button.last_tick = HAL_GetTick();
				Pickup_state = Ongoing;
				Pickup_mode = Overturn;
        	}
    	}

		if(Pickup_state == Finished)
		{
			CanTransmit_DJI_5678(&hcan1,
					hDJI[4].speedPID.output,
					hDJI[5].speedPID.output,
					hDJI[6].speedPID.output,
					hDJI[7].speedPID.output);
		}
		else if(Pickup_state == Ongoing) // 开始取环
		{
			// 取环
			switch(Pickup_mode)
			{
				case Motionless:
					Pickup_state = Finished;  
					break;
				case Overturn:
					if(hDJI[5].posPID.fdb > OverturnAngle - 1)
					{
						Pickup_mode = Extend;
						osDelay(20);
					}
					break;
				case Extend:
					if(hDJI[4].posPID.fdb > ExtendAngle - 1)
					{
						Pickup_mode = Claw_extend;
						osDelay(20);
					}
					break;
				case Claw_extend:
					if(hDJI[6].posPID.fdb > ClawAngle + 1)
					{
						Pickup_mode = Retract;
						osDelay(20);
					}
					break;
				case Retract:
					if(hDJI[4].posPID.fdb < 1)
					{
						Pickup_mode = Overturn_back;
						osDelay(20);
					}
					break;
				case Overturn_back:
					if(hDJI[5].posPID.fdb < 1)
					{
						Pickup_mode = Claw_retract;
						osDelay(20);
					}
					break;
				case Claw_retract:
					if(hDJI[6].posPID.fdb < 1)
					{
						Pickup_state = Finished;
					} 
					break;
					
			}
			CanTransmit_DJI_5678(&hcan1,
					hDJI[4].speedPID.output,
					hDJI[5].speedPID.output,
					hDJI[6].speedPID.output,
					hDJI[7].speedPID.output);
		}
		osDelay(1);
	}
}

void PickUpTestTask(void const *argument)
{
	PickUpInit();//取环机构初始化
    osDelay(20);

	Pickup_state = Ongoing;
	Pickup_mode = Overturn;
    for (;;) 
	{
		// if(Raw_Data.wheel < -550)
		// {
		// 	Pickup_state = Ongoing;
		// 	Pickup_mode = Overturn;
		// }
		if(Pickup_state == Finished)
		{
			CanTransmit_DJI_5678(&hcan1,
					hDJI[4].speedPID.output,
					hDJI[5].speedPID.output,
					hDJI[6].speedPID.output,
					hDJI[7].speedPID.output);
		}
		else if(Pickup_state == Ongoing) // 开始取环
		{
			// 取环
			switch(Pickup_mode)
			{
				case Motionless:
					Pickup_state = Finished;  
					break;
				case Overturn:
					if(hDJI[5].posPID.fdb < OverturnAngle + 1)
					{
						Pickup_mode = Extend;
						osDelay(1);
					}
					break;
				case Extend:
					if(angMax==90)
					{
						Pickup_mode = Claw_extend;
						osDelay(1);
					}
					break;
				case Claw_extend:
					if(hDJI[6].posPID.fdb < ClawAngle + 1)
					{
						Pickup_mode = Retract;
						osDelay(1);
					}
					break;
				case Retract:
					if(hDJI[4].posPID.fdb < 2000)
					{
						Pickup_mode = Overturn_back;
						osDelay(1);
					}
					break;
				case Overturn_back:
					if(hDJI[5].posPID.fdb > 180)
					{
						Pickup_mode = Claw_retract;
						osDelay(2);
					}
					break;
				case Claw_retract:
					if(hDJI[6].posPID.fdb > -1)
					{
						Pickup_state = Finished;
						osDelay(1);
					} 
					break;
			}
			CanTransmit_DJI_5678(&hcan1,
					hDJI[4].speedPID.output,
					hDJI[5].speedPID.output,
					hDJI[6].speedPID.output,
					hDJI[7].speedPID.output);
		}
		static int led_count = 0;
		if(led_count++ > 100)
		{
			led_count = 0;
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
		}
		osDelay(1);
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

void PickUpInit()
{
    CANFilterInit(&hcan1);
    hDJI[4].motorType = M3508; // 丝杠伸缩
    hDJI[5].motorType = M3508; // 翻滚
    hDJI[6].motorType = M2006; // 爪子

    DJI_Init(); // 大疆电机初始化
}