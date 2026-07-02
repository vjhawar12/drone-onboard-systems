#include "servo.h"
#include "stm32g0b1xx.h"
#include "stm32g0xx_ll_tim.h"
#include "uart.h"

int release_busy, retract_busy;

void lock() {
    const char *buffer = "MCU: Locking ...";
    if (UART_tx(*buffer) != ERR_TX) {}
    release_busy = 1;
    retract_busy = 1;
    const char *buffer2 = "MCU: Locked";
    if (UART_tx(*buffer2) != ERR_TX) {}
}

void arm() {
    const char *buffer = "MCU: Arming ...";
    if (UART_tx(*buffer) != ERR_TX) {}
    release_busy = 0;
    retract_busy = 0;
    const char *buffer2 = "MCU: Armed";
    if (UART_tx(*buffer2) != ERR_TX) {}
}

void release() {
    const char *buffer = "MCU: Releasing ...";
    if (UART_tx(*buffer) != ERR_TX) {}
    // move from all the way left to all the way right
    // 2 ms pulse
    LL_TIM_OC_SetCompareCH1(TIM1, 2000);
    release_busy = 1;
    const char *buffer2 = "MCU: Released";
    if (UART_tx(*buffer2) != ERR_TX) {}
}

void retract() {
    // move from all the way right to all the way left 
    // 1 ms pulse
    const char *buffer = "MCU: Retracting ...";
    if (UART_tx(*buffer) != ERR_TX) {}
    LL_TIM_OC_SetCompareCH1(TIM1, 1000);
    retract_busy = 1;
    const char *buffer2 = "MCU: Retracted";
    if (UART_tx(*buffer2) != ERR_TX) {}
}