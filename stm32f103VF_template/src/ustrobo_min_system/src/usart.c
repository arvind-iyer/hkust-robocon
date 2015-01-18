#include "usart.h"

u8 USART_QUEUE[COMn][USART_DEQUE_SIZE] = {{0}};

void usart_init(COM_TypeDef COMx, u32 baudrate)
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
		case COM2:
		case COM3:
		case COM4:
		case COM5:
			RCC_APB1PeriphClockCmd(usart->USART_RCC, ENABLE);
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
	
	/* Enables the USART transmission complete interrupt */
	USART_ITConfig(usart->USART,USART_IT_TC,ENABLE);	
	
	/* Initialize the deque structure */
	(usart->deque).head = 0;
	(usart->deque).tail = 0;
	(usart->deque).length = USART_DEQUE_SIZE;
	(usart->deque).queue = USART_QUEUE[COMx];
	
}

void usart_rx_init(COM_TypeDef COMx, void (*handler)(u8 rx_data))
{
	/* Enables the USART receive interrupt */
	USART_ITConfig(USART_DEF[COMx].USART,USART_IT_RXNE,ENABLE);	
	USART_DEF[COMx].rx_handler = handler;
}

/*** TX ***/
u8 usart_tx_dequeue(COM_TypeDef COMx)
{
	if (USART_GetFlagStatus(USART_DEF[COMx].USART, USART_FLAG_TC) == SET) {
		/* If USART TX is available (transmission complete flag is set) */
		USART_DEQUE* deque = &USART_DEF[COMx].deque;
		USART_SendData(USART_DEF[COMx].USART, deque->queue[deque->head]);
		deque->head = (deque->head + 1) / deque->length; 
		return 1;
	} else { 
		/* Wait until the USART_FLAG_TC interrupt is called (NO WHILE-LOOP) */
		return 0;
	}
}

u8 usart_tx_enqueue(COM_TypeDef COMx, u8 byte)
{
	if (USART_GetFlagStatus(USART_DEF[COMx].USART, USART_FLAG_TC) != RESET) {
		USART_SendData(USART_DEF[COMx].USART, byte);
	} else {
		USART_DEQUE* deque = &USART_DEF[COMx].deque;

		if ((deque->tail + 1) % deque->length == deque->head) {
			// FULL QUEUE
			return 0;
		} else {
			// Available
			deque->queue[deque->tail] = byte;
			deque->tail = (deque->tail + 1) % deque->length;
			usart_tx_dequeue(COMx);
			return 1;
		}
		
	}
}

u8 usart_tx_byte(COM_TypeDef COMx, u8 byte)
{
	return usart_tx_enqueue(COMx, byte);
}

void USARTx_IRQHandler(COM_TypeDef COMx) 
{
	USART_TypeDef* const usart = USART_DEF[COMx].USART;
	
	// Tranmission complete interrupt
	if (USART_GetITStatus(usart, USART_IT_TC) != RESET) {
		
	}
	
	// Receive interrupt
	if (USART_GetITStatus(usart, USART_IT_RXNE) != RESET) {
		
	}

}

void USART1_IRQHandler(void)
{
	USARTx_IRQHandler(COM1);
}

void USART2_IRQHandler(void)
{
	USARTx_IRQHandler(COM2);
}

