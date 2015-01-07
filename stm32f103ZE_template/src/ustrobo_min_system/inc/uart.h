#ifndef __UART_H
#define __UART_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

#include <stdio.h>
#include <stdarg.h>

typedef enum 
{
	COM_NULL = -1,		//disabled
    COM1 = 0,   //usart1
    COM2 = 1,	//usart2
    COM3 = 2,   //usart3
    COM4 = 3,	//uart4
    COM5 = 4    //uart5
} COM_TypeDef;

#define COMn 5 

// Definition for USART1 
#define COM1_CLK                    RCC_APB2Periph_USART1
#define COM1_TX_PIN                 GPIO_Pin_9
#define COM1_TX_GPIO_PORT           GPIOA
#define COM1_TX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define COM1_RX_PIN                 GPIO_Pin_10
#define COM1_RX_GPIO_PORT           GPIOA
#define COM1_RX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define COM1_IRQn                   USART1_IRQn

// Definition for USART2 
#define COM2_CLK                    RCC_APB1Periph_USART2
#define COM2_TX_PIN                 GPIO_Pin_2
#define COM2_TX_GPIO_PORT           GPIOA
#define COM2_TX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define COM2_RX_PIN                 GPIO_Pin_3
#define COM2_RX_GPIO_PORT           GPIOA
#define COM2_RX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define COM2_IRQn                   USART2_IRQn

// Definition for USART3 
#define COM3_CLK                    RCC_APB1Periph_USART3
#define COM3_TX_PIN                 GPIO_Pin_8
#define COM3_TX_GPIO_PORT           GPIOD
#define COM3_TX_GPIO_CLK            RCC_APB2Periph_GPIOD
#define COM3_RX_PIN                 GPIO_Pin_9
#define COM3_RX_GPIO_PORT           GPIOD
#define COM3_RX_GPIO_CLK            RCC_APB2Periph_GPIOD
#define COM3_IRQn                   USART3_IRQn

// Definition for UART4 
#define COM4_CLK                    RCC_APB1Periph_UART4
#define COM4_TX_PIN                 GPIO_Pin_10
#define COM4_TX_GPIO_PORT           GPIOC
#define COM4_TX_GPIO_CLK            RCC_APB2Periph_GPIOC
#define COM4_RX_PIN                 GPIO_Pin_11
#define COM4_RX_GPIO_PORT           GPIOC
#define COM4_RX_GPIO_CLK            RCC_APB2Periph_GPIOC
#define COM4_IRQn                   UART4_IRQn

// Definition for UART5
#define COM5_CLK                    RCC_APB1Periph_UART5
#define COM5_TX_PIN                 GPIO_Pin_12
#define COM5_TX_GPIO_PORT           GPIOC
#define COM5_TX_GPIO_CLK            RCC_APB2Periph_GPIOC
#define COM5_RX_PIN                 GPIO_Pin_2
#define COM5_RX_GPIO_PORT           GPIOD
#define COM5_RX_GPIO_CLK            RCC_APB2Periph_GPIOD
#define COM5_IRQn                   UART5_IRQn

extern USART_TypeDef* COM_USART[COMn];
extern COM_TypeDef printf_COMx;

// COM1=USART1, COM2=USART2; ..., COM5=UART5
void uart_init(COM_TypeDef COM, u32 br);
void uart_interrupt(COM_TypeDef COM);

void uart_printf_enable(COM_TypeDef COM);
void uart_printf_disable(void);

void uart_tx_byte(COM_TypeDef COM, const uint8_t data);
void uart_tx(COM_TypeDef COM, const uint8_t * tx_buf, ...);

#endif		/* __UART_H */
