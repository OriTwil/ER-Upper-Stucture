#ifndef PICK_UP_H
#define PICK_UP_H

#include "user_main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "user_calculate.h"
#include "wtr_can.h"
#include "wtr_dji.h"
#include <math.h>
#include "user_config.h"

void PickUpInit();
void PickUpTask(void const *argument);
void PickUpTestTask(void const *argument);
void PickUpTaskStart(mavlink_controller_t *controldata);
void DeadBand(double x, double y, double *new_x, double *new_y, double threshould);


typedef struct
{
    uint32_t last_tick;
    uint32_t button_min_time;
}Button;

extern Button button;
#endif
