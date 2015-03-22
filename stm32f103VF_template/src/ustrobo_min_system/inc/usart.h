#ifndef	__USART_H
#define	__USART_H

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"

#include <stdio.h>
#include <stdarg.h>

#define	COMn	5

#define	USART_DEQUE_SIZE	1000
typedef enum {
  COM_NULL = -1,
	COM1 = 0,
	COM2 = 1,
	COM3 = 2,
	COM4 = 3,
	COM5 = 4
} COM_TypeDef;

typedef struct {
	u16 head;
	u16 tail;
	u16 length;
	u8* queue;
} USART_DEQUE;

typedef struct {
	USART_TypeDef* USART;
	u32 USART_RCC;
	// TX
	GPIO_TypeDef* TX_PORT;
	u16 TX_PIN;
	u32 TX_RCC;	
	// RX
	GPIO_TypeDef* RX_PORT;
	u16 RX_PIN;
	u32 RX_RCC;	
	
	// IRQn
	IRQn_Type IRQn;
	
	// TX Queue
	USART_DEQUE deque;
	void (*rx_handler)(u8 rx_data);
} USART_TYPE;

// Backward compatible extern variable
//extern USART_TypeDef* COM_USART[COMn];

static USART_TYPE USART_DEF[COMn] = {
	{USART1, RCC_APB2Periph_USART1, 
	GPIOA, GPIO_Pin_9, RCC_APB2Periph_GPIOA,
	GPIOA, GPIO_Pin_10, RCC_APB2Periph_GPIOA,
	USART1_IRQn}
	
	,{USART2, RCC_APB1Periph_USART2, 
	GPIOA, GPIO_Pin_2, RCC_APB2Periph_GPIOA,
	GPIOA, GPIO_Pin_3, RCC_APB2Periph_GPIOA,
	USART2_IRQn}
	
	,{USART3, RCC_APB1Periph_USART3, 
	GPIOB, GPIO_Pin_10, RCC_APB2Periph_GPIOB,
	GPIOB, GPIO_Pin_11, RCC_APB2Periph_GPIOB,
	USART3_IRQn}
	
	,{UART4, RCC_APB1Periph_UART4, 
	GPIOC, GPIO_Pin_10, RCC_APB2Periph_GPIOC,
	GPIOC, GPIO_Pin_11, RCC_APB2Periph_GPIOC,
	UART4_IRQn}
	
	,{UART5, RCC_APB1Periph_UART5, 
	GPIOC, GPIO_Pin_12, RCC_APB2Periph_GPIOC,
	GPIOD, GPIO_Pin_2, RCC_APB2Periph_GPIOD,
	UART5_IRQn}
};

void uart_init(COM_TypeDef COMx, u32 baudrate);
void uart_printf_enable(COM_TypeDef COMx);
void uart_printf_disable(void);

void uart_rx_init(COM_TypeDef COMx, void (*handler)(u8 rx_data));
u16 uart_tx_dequeue(COM_TypeDef COMx);
u8 uart_tx_enqueue(COM_TypeDef COMx, u8 byte);
void uart_tx_byte(COM_TypeDef COMx, uc8 data);
void uart_tx(COM_TypeDef COM, uc8 * tx_buf, ...);


#endif /* __USART_H */
