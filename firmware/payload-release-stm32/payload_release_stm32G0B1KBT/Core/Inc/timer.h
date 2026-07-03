#ifndef TIMER_H
#define TIMER_H

#include "stm32g0xx_hal.h"
#include <stdint.h>

extern volatile int tick_high;
extern volatile int tick_low;
extern volatile int time_diff;

uint32_t micros();
uint32_t millis();
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif