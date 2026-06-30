#include "timer.h"
#include "stdint.h"

uint32_t millis() {
    return systick_counter;
}
