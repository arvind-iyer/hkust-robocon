#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include <stdio.h>

void uart_init(u32 baudrate);

void Print(const char* pstr, ...);
#endif


