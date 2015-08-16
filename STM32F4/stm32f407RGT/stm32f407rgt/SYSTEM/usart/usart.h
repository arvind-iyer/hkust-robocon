#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"

#include <stdio.h>
#include <stdbool.h>

void uart_init(u32 baudrate);
void usart_tx_one_byte(char data);
void usart_print(const char* pstr, ...);
void usart_rx();


#endif


