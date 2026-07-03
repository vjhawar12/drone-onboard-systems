#include "stm32g0b1xx.h"
#include "stm32g0xx_ll_usart.h"
#include "stdint.h"
#include "timer.h"
#include <stdint.h>
#include "uart.h"
#include "errors.h"
#include "SEGGER_RTT.h"

app_err_t __UART_tx(const char byte) {
    LL_USART_TransmitData8(USART1, byte);
    uint32_t start = millis();
    while (!LL_USART_IsActiveFlag_TC(USART1)) {        
        // if timeout return error
        if (millis() - start >= UART_TIMEOUT_MS) {
            return ERR_TX;
        }
    }
    return NONE;
}

app_err_t UART_tx(const char byte) {
    if (__UART_tx(byte) != NONE) {
        SEGGER_RTT_printf(0, "Error with UART tx in File %s Line %d", __FILE__, __LINE__); 
        return ERR_TX;
    } 
    return NONE;
}

app_err_t UART_tx_line(const char *buffer, int length) {
    for (int i = 0; i < length; i++) {
        if (UART_tx(buffer[i]) == ERR_TX) {
            return ERR_TX;
        }
    }
    return NONE;
}

app_err_t __UART_rx(char *buffer) {
    uint32_t start = millis();
    while (!LL_USART_IsActiveFlag_RXNE_RXFNE(USART1)) {
        // if timeout return error
        if (millis() - start >= UART_TIMEOUT_MS) {
            return ERR_RX;
        }
    }
    *buffer = LL_USART_ReceiveData8(USART1);
    return NONE;
}

app_err_t UART_rx(char *buffer) {
    if (__UART_rx(buffer) != NONE) {
        SEGGER_RTT_printf(0, "Error with UART rx in File %s Line %d", __FILE__, __LINE__); 
        return ERR_RX;
    } 
    return NONE;
}

app_err_t UART_rx_line(char *buffer, int length) {
    if (length == -1) {
        int i = 0;
        while (UART_rx(&buffer[i]) != ERR_RX) {
            if (buffer[i] == '\n') {
                return NONE;
            }
            i++;
        }
        return ERR_RX;
    } else {
        for (int i = 0; i < length; i++) {
            if (UART_rx(&buffer[i]) == ERR_RX) {
                return ERR_RX;
            }
        }
        return NONE;
    } 

}