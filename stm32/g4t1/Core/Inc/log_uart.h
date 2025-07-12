/**
 * Martin Egli
 * 2025-07-05
 * log uart module
 */
#ifndef _LOG_UART_H_ 
#define _LOG_UART_H_ 

#include "project.h"
#include "str_buf.h"

#define LOG_UART_NAME_SIZE (8)
#define LOG_UART_LOG_BUF_SIZE (128)
typedef struct 
{
    uint16_t uart;
    char name[LOG_UART_NAME_SIZE]; /// deep copy
    char log_buf[LOG_UART_LOG_BUF_SIZE];
    uint16_t state;
} log_uart_t;
#define LOG_UART_STATE_ON (0x0001)


uint16_t log_uart_init(log_uart_t *l, char *name);
uint16_t log_uart_enable(log_uart_t *l);
uint16_t log_uart_disable(log_uart_t *l);
uint16_t log_uart_string(log_uart_t *l, char *str);
uint16_t log_uart_value_float(log_uart_t *l, char *value_name, float value);

#endif // _LOG_UART_H_
