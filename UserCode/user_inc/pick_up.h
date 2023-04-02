#ifndef PICK_UP_H
#define PICK_UP_H

#include "usermain.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "Caculate.h"
#include "wtr_can.h"
#include "DJI.h"
#include <math.h>
#include "ctrl_data_sender.h"
#include "beep.h"

#define Ongoing 1
#define Finished 0
#define OverturnAngle (-728 - 1)
#define ExtendAngle (5000 + 1) 
#define ClawAngle (-35 - 1)

void PickUpInit();
void PickUpTask(void const *argument);
void PickUpTestTask(void const *argument);
void PickUpTaskStart(mavlink_controller_t *controldata);
void DeadBand(double x, double y, double *new_x, double *new_y, double threshould);

typedef enum
{
	Motionless,
	Overturn,
	Extend,
	Claw_extend,
	Retract,
	Overturn_back,
	Claw_retract
}Mode;

typedef struct
{
    uint32_t last_tick;
    uint32_t button_min_time;
}Button;

extern bool Pickup_state;
extern Mode Pickup_mode;
extern Button button;
#endif
