#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "buzzer.h"

// Backward compatible extern variable
USART_TypeDef* COM_USART[COMn] = {
  #if (COMn >= 3)
  USART1, 
  USART2, 
  USART3, 
  #endif
  
  #if (COMn >= 5)
  UART4, 
  UART5
  #endif
}; 
u8 USART_QUEUE[COMn][USART_DEQUE_SIZE] = {{0}};

void uart_init(COM_TypeDef COMx, u32 baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_TYPE* usart = &USART_DEF[COMx];
	
	// Enable USART RCC Clock
	RCC_APB2PeriphClockCmd(usart->TX_RCC | usart->RX_RCC | RCC_APB2Periph_AFIO, ENABLE);
	
	switch (COMx) {
		case COM1:
			RCC_APB2PeriphClockCmd(usart->USART_RCC, ENABLE);
			break;
		case COM2:
		case COM3:
		case COM4:
		case COM5:
			RCC_APB1PeriphClockCmd(usart->USART_RCC, ENABLE);
			break;
	}
	
	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = usart->TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(usart->TX_PORT, &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = usart->RX_PIN;
	GPIO_Init(usart->RX_PORT, &GPIO_InitStructure);

	/* USART configuration */
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(usart->USART, &USART_InitStructure);
	USART_Cmd(usart->USART, ENABLE);
	
	
	/* NVIC configuration */
	NVIC_InitStructure.NVIC_IRQChannel = usart->IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enables the USART receive interrupt */
	USART_ITConfig(usart->USART,USART_IT_RXNE,ENABLE);	
	
	/* Initialize the deque structure */
	(usart->deque).head = 0;
	(usart->deque).tail = 0;
	(usart->deque).length = USART_DEQUE_SIZE;
	(usart->deque).queue = USART_QUEUE[COMx];
	
}



void uart_rx_init(COM_TypeDef COMx, void (*handler)(u8 rx_data))
{
	/* Enables the USART receive interrupt */
	USART_TYPE* usart = &USART_DEF[COMx];
	USART_ITConfig(usart->USART,USART_IT_RXNE,ENABLE);	
	usart->rx_handler = handler;
}

/**
	* @brief Check if the USART_TX queue is empty
	* @param None
	* @retval True if the queue is empty
	*/
u8 uart_tx_queue_empty(COM_TypeDef COMx)
{
	USART_TYPE* usart = &USART_DEF[COMx];
	return (usart->deque).head == (usart->deque).tail;
}


/**
	* @brief	Get the current USART_TX queue size
	* @param 	None
	* @retval	The current queue size (0 to USART_TX_QUEUE_MAX_SIZE-1)
	*/
u16 uart_tx_queue_size(COM_TypeDef COMx)
{
	USART_TYPE* usart = &USART_DEF[COMx];
	s16 size = (usart->deque).tail - (usart->deque).head;
	if (size < 0) {size += (usart->deque).length;}
	return (u16) size;
}


/** 
	* @brief Add a new tx message to the usart Tx queue
	* @param msg: The usart message that will be added
	* @retval 0: Fail to enqueue due to the exceeding size, 1: Successfully enqueued
	*/
u8 uart_tx_enqueue(COM_TypeDef COMx, uc8 data){
	USART_TYPE* usart = &USART_DEF[COMx];
	USART_DEQUE* deque_cur= &(usart->deque);

	u8 queue_full=0;
	if(USART_GetFlagStatus(usart->USART, USART_FLAG_TC) != RESET){
		USART_ClearFlag(usart->USART, USART_FLAG_TC);
		USART_SendData(usart->USART ,data);
	}
	else{
		if ((deque_cur->tail+1)%deque_cur->length ==deque_cur->head ){
			queue_full=1;
		}
		else{
			deque_cur->queue[deque_cur->tail]=data;
			deque_cur->tail = (deque_cur->tail + 1) % deque_cur->length;
			queue_full=0;
		}
	}
	uart_tx_dequeue(COMx);
	return !queue_full;
}

/**
	* @brief	Process and transfer ONE usart message in the queue and dequeue
	* @param 	None
	*	@retval True if the queue is not empty after dequeue
	*/
u16 uart_tx_dequeue(COM_TypeDef COMx)
{
	USART_TYPE* usart = &USART_DEF[COMx];
	USART_DEQUE* deque = &(usart->deque);
	/* If USART TX is available  */
	if(USART_GetFlagStatus(USART_DEF[COMx].USART, USART_FLAG_TXE)==SET){
		USART_ClearFlag(usart->USART, USART_FLAG_TXE);
		if(!uart_tx_queue_empty(COMx)){  // Non-empty deque
			USART_SendData(usart->USART, deque->queue[deque->head]);
			deque->head = (deque->head + 1) % uart_tx_queue_size(COMx);
			return 1;
		}else{
		return 0;
		}
	}else{
	return 0;
	}
}

void uart_tx_byte(COM_TypeDef COMx, uc8 data)
{
	//uart_tx_enqueue(COMx, data);
  while (USART_GetFlagStatus(USART_DEF[COMx].USART, USART_FLAG_TXE) == RESET);
  USART_SendData(USART_DEF[COMx].USART, data);
}


void uart_tx(COM_TypeDef COMx, uc8 * tx_buf, ...)
{
	va_list arglist;
	u8 buf[40], *fp;
	va_start(arglist, tx_buf);
	vsprintf((char*)buf, (const char*)tx_buf, arglist);
	va_end(arglist);
	
	fp = buf;
	while (*fp)
		uart_tx_byte(COMx,*fp++);
}


u16 uart_tx_queue_head(COM_TypeDef COMx){
	USART_DEQUE* deque = &USART_DEF[COMx].deque;
	return deque->head;
}
u16 uart_tx_queue_tail(COM_TypeDef COMx){
	USART_DEQUE* deque = &USART_DEF[COMx].deque;
	return deque->tail;
}


u16 uart_tx_queue_cur_length(COM_TypeDef COMx){
	USART_DEQUE* deque = &USART_DEF[COMx].deque;
	return deque->tail - deque->head ;
}


void USART_Tx_IRQHandler(COM_TypeDef COMx) 
{
	USART_TYPE* usartx = &USART_DEF[COMx];
	USART_TypeDef* const usart = usartx->USART;
	if (USART_GetITStatus(usart, USART_IT_TXE) != RESET)
	{
		USART_ClearITPendingBit(usart, USART_IT_TXE);
		uart_tx_dequeue(COMx);
	}
}

void USART_Rx_IRQHandler(COM_TypeDef COMx) {
	USART_TYPE* usartx = &USART_DEF[COMx];
	USART_TypeDef* const usart = usartx->USART;
	if (USART_GetITStatus(usart, USART_IT_RXNE) != RESET) {
		USART_ClearITPendingBit(usart, USART_IT_RXNE);
		if (usartx->rx_handler != 0) {
			usartx->rx_handler(USART_ReceiveData(usart));
		}
	}
}

void USART1_IRQHandler(void) {
	USART_Rx_IRQHandler(COM1);
}

void USART2_IRQHandler(void) {
	USART_Rx_IRQHandler(COM2);
}

void USART3_IRQHandler(void) {
	USART_Rx_IRQHandler(COM3);
}

/*** Caution: 4 and 5 MUST be UART rather than USART ***/
void UART4_IRQHandler(void) {
	USART_Rx_IRQHandler(COM4);
}

void UART5_IRQHandler(void) {
	USART_Rx_IRQHandler(COM5);
}

