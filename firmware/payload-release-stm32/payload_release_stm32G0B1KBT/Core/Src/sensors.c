#include "stm32g0b1xx.h"
#include "stm32g0xx_ll_gpio.h"
#include "timer.h"
#include "sensors.h"
#include "SEGGER_RTT.h"

void ultrasonic_trigger() {
    int start = micros();
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
    while (micros() - start < 10) {}
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
}

app_err_t __ultrasonic_distance_cm(uint16_t *buffer) {
    int distance = time_diff / 58;
    if (distance <= 0) {
        return ERR_ULTRASONIC;
    }
    *buffer = distance;
    return NONE;   
}

app_err_t ultrasonic_distance_cm(uint16_t *buffer) {
    if (__ultrasonic_distance_cm(buffer) != NONE) {
        SEGGER_RTT_printf(0, "Error retrieving ultrasonic distance in File %s Line %d ", __FILE__, __LINE__); 
        return ERR_ULTRASONIC;
    } 
    return NONE;
}