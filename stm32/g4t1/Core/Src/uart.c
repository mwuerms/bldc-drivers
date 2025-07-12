/**
 * Martin Egli
 * 2025-07-02
 * uart module
 */

#include "uart.h"

uint16_t uart_init(uart_t *u, USART_TypeDef *uart) {
    if(u == NULL) {
        // error, invalid uart
        return false;
    }
    if(uart == NULL) {
        // error, invalid uart
        return false;
    }
    u->uart = uart;
    return true;
}
uint16_t uart_send_string_blocking(uart_t *u, char *str) {
    if(u == NULL) {
        // error, invalid uart
        return false;
    }
    while(*str != 0) {
        // Wait until transmit data register is empty
		while(LL_USART_IsActiveFlag_TXE(u->uart) == 0);
		// Send a character
		LL_USART_TransmitData8(u->uart, *str++);
    }
    // Wait for TC (Transmission Complete) flag to ensure last byte is sent
	while(!LL_USART_IsActiveFlag_TC(u->uart));
    return true;
}
