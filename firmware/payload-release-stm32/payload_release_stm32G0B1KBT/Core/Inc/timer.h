#ifndef TIMER_H
#define TIMER_H

#include "stm32g0xx_hal.h"
#include <stdint.h>

extern int tick_high;
extern int tick_low;
extern int time_diff;

uint32_t micros();
uint32_t millis();
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif