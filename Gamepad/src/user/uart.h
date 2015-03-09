#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"
#include <stdio.h>
#include "stm32f10x_usart.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdarg.h>

typedef enum 
{
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
#define COM3_TX_PIN                 GPIO_Pin_10	
#define COM3_TX_GPIO_PORT           GPIOB
#define COM3_TX_GPIO_CLK            RCC_APB2Periph_GPIOB
#define COM3_RX_PIN                 GPIO_Pin_11
#define COM3_RX_GPIO_PORT           GPIOB
#define COM3_RX_GPIO_CLK            RCC_APB2Periph_GPIOB
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
extern USART_TypeDef* printf_UARTx;

void uart_init(COM_TypeDef COM);    	//COM1=USART1,COM2=USART2,.....,COM5=UART5
void uart_interrupt(COM_TypeDef COM);	//COM1=USART1,COM2=USART2,.....,COM5=UART5
     
void uart_tx_byte(USART_TypeDef* UARTx, const uint8_t data);
void uart_tx(USART_TypeDef* UARTx, const uint8_t * tx_buf, ...);
void printf_switch_uart(USART_TypeDef* UARTx);//printf_switch_uart(USART1);

void uart_printf_enable(void);
void uart_printf_disable(void);

#ifdef __cplusplus
}
#endif

#endif		/* __UART_H */
