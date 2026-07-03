#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_TIMEOUT_MS 10

typedef enum uart_error_t {
    ERR_TX,
    ERR_RX,
    NONE,
} uart_error_t;

uart_error_t __UART_tx(uint8_t byte);
void UART_tx(uint8_t byte);
uart_error_t __UART_rx(char *byte);
void UART_rx(char *byte);

#endif