#ifndef READ_UART_H
#define READ_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32g0b1xx.h"
#include "stm32g0xx_ll_usart.h"

static inline bool read_character(uint8_t *byte) {
    if (LL_USART_IsActiveFlag_RXNE(USART1)) {
        *byte = LL_USART_ReceiveData8(USART1); 
        return true;
    }
    return false;
}

static inline bool write_character(uint8_t *byte) {
    LL_USART_TransmitData8(USART1, *byte); 
    while (LL_USART_IsActiveFlag_TXE(USART1)); 
    return true;
    // optional: return false if timeout
}

static inline bool write_string(uint8_t *string, size_t length) {
    bool ret = true;
    for (int c = 0; c < length; c++) {
        ret &= write_character(&string[c]); 
    }
    return ret; 
}

#endif