#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "errors.h"

#define UART_TIMEOUT_MS 10


app_err_t __UART_tx(const char byte);
app_err_t UART_tx(const char byte);
app_err_t UART_tx_line(const char *buffer, int length);
app_err_t __UART_rx(char *byte);
app_err_t UART_rx(char *byte);
app_err_t UART_rx_line(char *buffer, int length);

#endif