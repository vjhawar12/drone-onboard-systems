#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_TIMEOUT_MS 10

typedef enum uart_error_t {
    ERR_TX,
    ERR_RX,
    NONE,
} uart_error_t;

uart_error_t UART_tx(uint8_t byte);
uart_error_t UART_rx(uint8_t *byte);

#endif