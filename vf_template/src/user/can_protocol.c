#include "can_protocol.h"


CAN_MESSAGE CAN_Tx_Queue_Array[CAN_TX_QUEUE_SIZE];
CAN_QUEUE CAN_Tx_Queue = {0, 0, CAN_TX_QUEUE_SIZE, CAN_Tx_Queue_Array};

void can_tx_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(CAN_TX_RCC , ENABLE);
	CAN_TX_TIM->PSC = SystemCoreClock / 10000 - 1;		// Prescaler
	CAN_TX_TIM->ARR = 200;	// 20 ms
	CAN_TX_TIM->EGR = 1;
	CAN_TX_TIM->SR = 0;
	CAN_TX_TIM->DIER = 1;
	CAN_TX_TIM->CR1 = 1;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = CAN_TX_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

/** 
	* @brief Add a new tx message to the CAN Tx queue
	* @param msg: The can message that will be added
	* @retval 0: Fail to enqueue due to the exceeding size, 1: Successfully enqueued
	*/
u8 can_tx_enqueue(CAN_MESSAGE msg)
{
	if ((CAN_Tx_Queue.tail + 1) % CAN_Tx_Queue.length == CAN_Tx_Queue.head) {
		// Queue full
		return 0;
	}	else {
		CAN_Tx_Queue.queue[CAN_Tx_Queue.tail] = msg;
		CAN_Tx_Queue.tail = (CAN_Tx_Queue.tail + 1) % CAN_Tx_Queue.length;
		return 1;
	}
}

/**
	* @brief Process and transfer ONE can message in the queue and dequeue
	*	@retval True if the queue is not empty
	*/

u8 can_tx_dequeue(void)
{
	if (CAN_Tx_Queue.head == CAN_Tx_Queue.tail) {
		return 0;
	} else {
		CAN_MESSAGE msg = CAN_Tx_Queue.queue[CAN_Tx_Queue.head];
		CanTxMsg TxMsg;
		u8 data_length = msg.length;
		
		TxMsg.StdId = msg.id;
		TxMsg.ExtId = 0x00;
		TxMsg.RTR = CAN_RTR_DATA;
		TxMsg.IDE = CAN_ID_STD;
		TxMsg.DLC = data_length;
		
		// Copy the data array
		while (data_length--) {
			TxMsg.Data[data_length] = msg.data[data_length];
		}

		if (can_tx(TxMsg)) {
			CAN_Tx_Queue.head = (CAN_Tx_Queue.head + 1) % CAN_Tx_Queue.length;
		}
		
		return 1;
	}
}

CAN_TX_IRQHander
{
	u16 retry = CAN_TX_DEQUEUE_TIMEOUT;
	TIM_ClearFlag(CAN_TX_TIM, TIM_FLAG_Update);
	TIM_ClearITPendingBit(CAN_TX_TIM, TIM_IT_Update);
	
	while(can_tx_dequeue() && retry--);
}





