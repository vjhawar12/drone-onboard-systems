#include "stm32g0b1xx.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_tim.h"
#include <stdint.h>
#include "timer.h"

uint32_t micros() {
    return LL_TIM_GetCounter(TIM2);
}

uint32_t millis() {
    return HAL_GetTick();
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        if (LL_TIM_IC_GetPolarity(TIM2, LL_TIM_CHANNEL_CH1) == LL_TIM_IC_POLARITY_RISING) {
            tick_high = LL_TIM_IC_GetCaptureCH1(TIM2); 
            LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_FALLING); 
        } else {
            tick_low = LL_TIM_IC_GetCaptureCH1(TIM2); 
            LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
            time_diff = tick_low - tick_high;
        }
    }
}