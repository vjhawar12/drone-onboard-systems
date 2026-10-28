#include "stm32g0b1xx.h"
#include "stm32g0xx_ll_tim.h"

// 20 ms period with 50 hz frequency

#define MAX_DEG 180
#define PERIOD_MS 20
#define FREQ_HZ 50
#define PW_MIN 1.0
#define PW_MAX 2.0
#define ARR (TIM1->ARR)
#define DUTY_MIN (PW_MIN / PERIOD_MS)
#define DUTY_MAX (PW_MAX / PERIOD_MS)
#define CCR_MIN (DUTY_MIN * ARR)
#define CCR_MAX (DUTY_MAX * ARR)
// 0 deg = 5% duty cycle = 5% * ARR = 0.05 * 19999 = 999
// 180 deg = 10% duty cycle = 0.1 * 19999 = 1999

// converts degrees to pulse pulse width between pulse_min for 0 deg 
// to pulse max for 180 deg and then to a fraction of ARR
int servo_move_to(uint16_t degrees) {
    if (degrees < MAX_DEG) {
        uint32_t ccr = CCR_MIN + (degrees / MAX_DEG) * (CCR_MIN / CCR_MAX); 
        return 0; 
    } else {
        return 1; 
    }
}

int servo_release() {
    return servo_move_to(180); 
}

int servo_retract() {
    return servo_move_to(0); 
}
