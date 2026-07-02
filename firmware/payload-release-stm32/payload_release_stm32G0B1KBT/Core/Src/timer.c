#include "stm32g0xx_hal.h"
#include <stdint.h>

uint32_t millis() {
    return HAL_GetTick();
}