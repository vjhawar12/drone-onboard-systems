#ifndef READ_UART_H
#define READ_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0b1xx.h"
#include "stm32g0xx_ll_usart.h"

bool read_character(uint8_t *byte) {
    if (LL_USART_IsActiveFlag_RXNE) {
        *byte = LL_USART_ReceiveData8(USART1); 
        return true;
    }
    return false;
}

#endif