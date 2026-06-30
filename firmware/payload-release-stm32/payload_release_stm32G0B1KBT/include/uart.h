#ifndef UART_H
#define UART_H

#include "stdint.h"
#define UART_TIMEOUT_MS 10

typedef enum UART_ERRORS {
    ERR_NONE,
    ERR_TIMEOUT_TX,
    ERR_TIMEOUT_RX
} uart_errors_t;

int uart_tx(uint8_t byte);
uint8_t uart_rx();

#endif
