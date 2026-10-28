#include "stm32g0b1xx.h"
#include "stm32g0xx_ll_usart.h"
#include <stdint.h>
#include "read_uart.h"

bool read_character(uint8_t *byte) {
    if (LL_USART_IsActiveFlag_RXNE) {
        *byte = LL_USART_ReceiveData8(USART1); 
        return true;
    }
    return false;
}