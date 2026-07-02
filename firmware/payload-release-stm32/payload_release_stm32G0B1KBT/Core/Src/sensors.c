#include "stm32g0b1xx.h"
#include "stm32g0xx_ll_gpio.h"
#include "timer.h"
#include "sensors.h"

void ultrasonic_trigger() {
    int start = micros();
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
    while (micros() - start < 10) {}
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
}

sensor_err_t ultrasonic_distance_cm(uint16_t *buffer) {
    int distance = time_diff / 58;
    if (distance <= 0) {
        return ERR_ULTRASONIC;
    }
    *buffer = distance;
    return NONE;   
}