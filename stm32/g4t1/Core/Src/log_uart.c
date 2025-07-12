/**
 * Martin Egli
 * 2025-07-05
 * log uart module
 */

#include "log_uart.h"
#include "str_buf.h"

#define uart_send(...)

uint16_t log_uart_init(log_uart_t *l, char *name) {
    if(l == NULL) {
        // error, invalid log
        return false;
    }
    // init uart
    l->uart = 1;
    str_buf_clear(l->name, LOG_UART_NAME_SIZE);
    str_buf_append_string(l->name, LOG_UART_NAME_SIZE, name);
    log_uart_disable(l);
    return true;
}

uint16_t log_uart_enable(log_uart_t *l) {
    if(l == NULL) {
        // error, invalid log
        return false;
    }
    l->state |=  LOG_UART_STATE_ON;
    return true;
}

uint16_t log_uart_disable(log_uart_t *l) {
    if(l == NULL) {
        // error, invalid log
        return false;
    }
    l->state &= ~LOG_UART_STATE_ON;
    return true;
}

uint16_t log_uart_string(log_uart_t *l, char *str) {
    if(l == NULL) {
        // error, invalid log
        return false;
    }
    if((l->state & LOG_UART_STATE_ON) == 0) {
        // is disabled
        return false;
    }
    return true;
    str_buf_clear(l->log_buf, LOG_UART_LOG_BUF_SIZE);
    str_buf_append_string(l->log_buf, LOG_UART_LOG_BUF_SIZE, l->name);
    str_buf_append_string(l->log_buf, LOG_UART_LOG_BUF_SIZE, ": ");
    str_buf_append_string(l->log_buf, LOG_UART_LOG_BUF_SIZE, str);
    uart_send(l->log_buf);
    return true;
}

uint16_t log_uart_value_float(log_uart_t *l, char *value_name, float value) {
    if(l == NULL) {
        // error, invalid log
        return false;
    }
    if((l->state & LOG_UART_STATE_ON) == 0) {
        // is disabled
        return false;
    }
    return true;
    str_buf_clear(l->log_buf, LOG_UART_LOG_BUF_SIZE);
    str_buf_append_string(l->log_buf, LOG_UART_LOG_BUF_SIZE, l->name);
    str_buf_append_string(l->log_buf, LOG_UART_LOG_BUF_SIZE, ": ");
    str_buf_append_string(l->log_buf, LOG_UART_LOG_BUF_SIZE, value_name);
    str_buf_append_string(l->log_buf, LOG_UART_LOG_BUF_SIZE, " ");
    str_buf_append_float(l->log_buf, LOG_UART_LOG_BUF_SIZE, value, 3);
    uart_send(l->log_buf);
    return true;
}
