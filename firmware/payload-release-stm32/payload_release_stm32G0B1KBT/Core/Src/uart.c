#include "stm32g0b1xx.h"
#include "stm32g0xx_ll_usart.h"
#include "stdint.h"
#include "timer.h"
#include <stdint.h>
#include "uart.h"

uart_error_t UART_tx(uint8_t byte) {
    LL_USART_TransmitData8(USART1, byte);
    uint32_t start = millis();
    while (!LL_USART_IsActiveFlag_TC(USART1)) {        
        // if timeout return error
        if (millis() - start == UART_TIMEOUT_MS) {
            return ERR_TX;
        }
    }
    return NONE;
}

uart_error_t UART_rx(uint8_t *byte) {
    uint32_t start = millis();
    while (!LL_USART_IsActiveFlag_RXNE_RXFNE(USART1)) {
        // if timeout return error
        if (millis() - start == UART_TIMEOUT_MS) {
            return ERR_RX;
        }
    }
    *byte = LL_USART_ReceiveData8(USART1);
    return NONE;
}