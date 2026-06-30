#include "uart.h"
#include "timer.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0b1xx.h"
#include "stdint.h"

int uart_tx(uint8_t byte) {
    LL_USART_TransmitData8(USART2, byte);
    uint32_t start = millis();
    while (!LL_USART_IsActiveFlag_TC(USART2)) {
        // wait for 10 ms, it not then return error
        if (millis() - start >= UART_TIMEOUT_MS) {
            return ERR_TIMEOUT_TX;
        }
    }
    return ERR_NONE;
}

uint8_t uart_rx() {
    uint8_t data = LL_USART_ReceiveData8(USART2);
    return data;
}



