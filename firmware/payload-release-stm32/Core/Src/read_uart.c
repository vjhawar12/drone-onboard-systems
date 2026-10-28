#include "stm32g0b1xx.h"
#include "stm32g0xx_ll_usart.h"
#include <stdint.h>
#include "read_uart.h"

void read_character(uint8_t *byte) {
    *byte = LL_USART_ReceiveData8(USART1); 
}