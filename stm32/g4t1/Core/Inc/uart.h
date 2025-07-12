/**
 * Martin Egli
 * 2025-07-02
 * uart module
 */

#ifndef _UART_MODULE_H_
#define _UART_MODULE_H_

#include "project.h"

typedef struct {
    USART_TypeDef *uart;
} uart_t;

uint16_t uart_init(uart_t *u, USART_TypeDef *uart);
uint16_t uart_send_string_blocking(uart_t *u, char *str);

#endif // _UART_MODULE_H_
