#ifndef FIRE_H
#define FIRE_H

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
#include "tim.h"

void FireTask(void const *argument);
void FireTestTask(void const *argument);
void FireTaskStart(mavlink_controller_t *controldata);
void FireInit();

#endif
