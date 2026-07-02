#ifndef TIMER_H
#define TIMER_H

#include "stm32g0xx_hal.h"
#include <stdint.h>

int tick_high = 0;
int tick_low = 0;
int time_diff = 0;

uint32_t micros();
uint32_t millis();
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif