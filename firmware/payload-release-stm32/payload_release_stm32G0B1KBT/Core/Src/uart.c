#include "stm32g0b1xx.h"
#include "stm32g0xx_ll_usart.h"
#include "stdint.h"
#include "timer.h"
#include <stdint.h>
#include "uart.h"
#include "SEGGER_RTT.h"

uart_error_t __UART_tx(uint8_t byte) {
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

void UART_tx(uint8_t byte) {
    if (__UART_tx(byte) != NONE) {
        SEGGER_RTT_printf(0, "Error with UART tx", __FILE__, __LINE__); 
    } 
}

uart_error_t __UART_rx(char *buffer) {
    uint32_t start = millis();
    while (!LL_USART_IsActiveFlag_RXNE_RXFNE(USART1)) {
        // if timeout return error
        if (millis() - start == UART_TIMEOUT_MS) {
            return ERR_RX;
        }
    }
    *buffer = LL_USART_ReceiveData8(USART1);
    return NONE;
}

void UART_rx(char *buffer) {
    if (__UART_rx(buffer) != NONE) {
        SEGGER_RTT_printf(0, "Error with UART rx", __FILE__, __LINE__); 
    } 
}