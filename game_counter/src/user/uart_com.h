#ifndef __UART_COM_H
#define __UART_COM_H
#include <stdbool.h>
#include <string.h>
#include "usart.h"
#include "timer.h"

#define UART_COM    COM1
#define UART_BR     9600
#define UART_RX_BUFFER_COUNT    50
#define UART_BUFFER_SIZE      120
#endif  /* __UART_COM_H */

void uart_com_init(void);
void uart_com_tx(const char * tx_buf, ...);
void uart_update(void);
