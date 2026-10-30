#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "stm32g0b1xx.h"
#include "stm32g0xx_ll_tim.h"
#include <stdint.h>

// 20 ms period with 50 hz frequency

#define MAX_DEG 180U
#define PERIOD_MS 20U
#define FREQ_HZ 50U
#define PW_MIN 1U
#define PW_MAX 2U
#define ARR (TIM1->ARR)
#define DUTY_MIN (PW_MIN / PERIOD_MS)
#define DUTY_MAX (PW_MAX / PERIOD_MS)
#define CCR_MIN (DUTY_MIN * ARR)
#define CCR_MAX (DUTY_MAX * ARR)

// 0 deg = 5% duty cycle = 5% * ARR = 0.05 * 19999 = 999
// 180 deg = 10% duty cycle = 0.1 * 19999 = 1999
// converts degrees to pulse pulse width between pulse_min for 0 deg 
// to pulse max for 180 deg and then to a fraction of ARR

static inline int servo_move_to(uint16_t degrees) {
    if (degrees <= MAX_DEG) {
        // avoiding floating point math by adding half the denomenator to numerator before dividing
        uint16_t ccr = CCR_MIN + ((degrees * (CCR_MAX - CCR_MIN + MAX_DEG/2)) / MAX_DEG); 
        // TIM1 is a 16 bit timer with a 16 bit CCR1 register, despite the C definition being uint32_t
        TIM1->CCR1 = ccr;
        return 0; 
    } else {    
        return 1; 
    }
}

static inline int servo_release() {
    return servo_move_to(180); 
}

static inline int servo_retract() {
    return servo_move_to(0); 
}

#endif