#include "can_protocol.h"


CAN_MESSAGE CAN_Tx_Queue_Array[CAN_TX_QUEUE_MAX_SIZE];
CAN_QUEUE CAN_Tx_Queue = {0, 0, CAN_TX_QUEUE_MAX_SIZE, CAN_Tx_Queue_Array};
u8 CAN_FilterCount = 0;

// Array storing all the handler function for CAN Rx (element id equals to filter id)
void (*CAN_Rx_Handlers[CAN_RX_FILTER_LIMIT])(CanRxMsg msg) ;


/**
	* @brief	Get the current CAN_TX queue head
	* @retval	The queue head ID (0 to CAN_TX_QUEUE_SIZE-1)
	*/
u16 can_tx_queue_head(void)
{
	return CAN_Tx_Queue.head;
}

/**
	* @brief	Get the current CAN_TX queue tail
	* @param 	None
	* @retval	The queue head ID (0 to CAN_TX_QUEUE_MAX_SIZE-1)
	*/
u16 can_tx_queue_tail(void)
{
	return CAN_Tx_Queue.tail;
}

/**
	* @brief	Get the current CAN_TX queue size
	* @param 	None
	* @retval	The current queue size (0 to CAN_TX_QUEUE_MAX_SIZE-1)
	*/
u16 can_tx_queue_size(void)
{
	s16 size = CAN_Tx_Queue.tail - CAN_Tx_Queue.head;
	if (size < 0) {size += CAN_Tx_Queue.length;}
	return (u16) size;
}

/**
	* @brief Check if the CAN_TX queue is empty
	* @param None
	* @retval True if the queue is empty
	*/
u8 can_tx_queue_empty(void)
{
	return CAN_Tx_Queue.head == CAN_Tx_Queue.tail;
}

/**
	* @brief	Get the number of empty (free) CAN mailboxes (Refer to the CAN_Transmit(...) function)
	* @param 	None
	* @retval	The number of empty CAN mailboxes (0 if no CAN mailbox available for anymore CAN Tx)
	*/
u8 can_empty_mailbox(void)
{
	return ((CANn->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
	+((CANn->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
	+ ((CANn->TSR&CAN_TSR_TME2) == CAN_TSR_TME2);
//	return (CAN_TransmitStatus(CANn, 0) == CAN_TxStatus_Ok)
//		+ (CAN_TransmitStatus(CANn, 1) == CAN_TxStatus_Ok)
//		+ (CAN_TransmitStatus(CANn, 2) == CAN_TxStatus_Ok);
}

/** 
	* @brief Add a new tx message to the CAN Tx queue
	* @param msg: The can message that will be added
	* @retval 0: Fail to enqueue due to the exceeding size, 1: Successfully enqueued
	*/
u8 can_tx_enqueue(CAN_MESSAGE msg)
{
	u8 queue_full = 0;
	if ((CAN_Tx_Queue.tail + 1) % CAN_Tx_Queue.length == CAN_Tx_Queue.head) {
		// Queue full
		queue_full = 1;
	}	else {
		CAN_Tx_Queue.queue[CAN_Tx_Queue.tail] = msg;
		CAN_Tx_Queue.tail = (CAN_Tx_Queue.tail + 1) % CAN_Tx_Queue.length;
		queue_full = 0;
		
	}
	
	can_tx_dequeue();
	return !queue_full;
}

/**
	* @brief	Process and transfer ONE can message in the queue and dequeue
	* @param 	None
	*	@retval True if the queue is not empty after dequeue
	*/
u8 can_tx_dequeue(void)
{
	if (!can_tx_queue_empty() && can_empty_mailbox() > 0) {
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
		
		// If there are still empty mailbox, dequeue again
		if (can_empty_mailbox() > 0) {
			can_tx_dequeue();
		}
		
		return 1;
	} else {
		return 0;
	}
}

/**
	* @brief Force clear the CAN_TX queue without process
	* @param None.
	* @retval None.
	*/
void can_tx_queue_clear(void)
{
	CAN_Tx_Queue.head = CAN_Tx_Queue.tail = 0;
}


/**
	* @brief The handler function of empty 
	*/
CAN_TX_IRQHander
{
	if (CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET)
	{
		// If all the mailboxes are empty
		CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
		can_tx_dequeue();
	}
}

/**
	* @brief Initialize the CAN_RX interrupt handler
	* @param None.
	* @retval None.
	*/
void can_rx_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*
	#ifdef  VECT_TAB_RAM  
	// Set the Vector Table base location at 0x20000000 
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  // VECT_TAB_FLASH  
	// Set the Vector Table base location at 0x08000000  
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif
	*/
	
	CAN_ITConfig(CANn, CAN_IT_FMP0, ENABLE);

	/* enabling interrupt */
	NVIC_InitStructure.NVIC_IRQChannel= CAN_Rx_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
	* @brief Add filter to the can data received (involves bitwise calculation)
	* @warning can only be called for 14 / 28 times. Check the function IS_CAN_FILTER_NUMBER for detail
	* @param id: 11-bit ID (0x000 to 0x7FF)
	* @param mask: 11-bit mask, corresponding to the 11-bit ID	(0x000 to 0x7FF)		
	* @param handler: function pointer for the corresponding CAN ID filter
	* @example can_rx_add_filter(0x000, 0x000) will receive CAN message with ANY ID
	* @example can_rx_add_filter(0x0CD, 0x7FF) will receive CAN message with ID 0xCD
	* @example can_rx_add_filter(0x0A0, 0x7F0) will receive CAN message with ID from 0xA0 to 0xAF
	* @example can_rx_add_filter(0x000, 0x7FA) will receive CAN message with ID from 0x00 to 0x03
	*/
void can_rx_add_filter(u16 id, u16 mask, void (*handler)(CanRxMsg msg))
{
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	mask = ((mask << 5) | 0x001F) & 0xFFFF;
	
	CAN_FilterInitStructure.CAN_FilterNumber = CAN_FilterCount;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = (id << 5) & 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = mask;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);	
	
	CAN_Rx_Handlers[CAN_FilterCount] = handler;
	
	++CAN_FilterCount;
}




/** 
	* @brief Interrupt for CAN Rx
	* @warning Use USB_LP_CAN_RX0_IRQHandler for HD, USB_LP_CAN1_RX0_IRQHandler for XLD / MD
	*/
CAN_Rx_IRQHandler
{
	if (CAN_GetITStatus(CANn, CAN_IT_FMP0) != RESET) {
		CanRxMsg RxMessage;
		CAN_ClearITPendingBit(CANn, CAN_IT_FMP0);
		CAN_Receive(CANn, CAN_FIFO0, &RxMessage);
		
		if(RxMessage.IDE == CAN_ID_STD) {
			u8 filter_id = RxMessage.FMI;
			if (filter_id < CAN_FilterCount && filter_id < CAN_RX_FILTER_LIMIT && CAN_Rx_Handlers[filter_id] != 0) {
				CAN_Rx_Handlers[filter_id](RxMessage);
			}
		}		
	}

}

/*** Protocol Encoding / Decoding function ***/
/**
	* @brief Convert one n-byte variable to n one-byte variable (ENCODE)
	* @param n: the nth byte 
	* @param num: the nth byte number (can be unsigned)
	* @retval The nth byte variable
	*/
u8 one_to_n_bytes(s32 num, u8 n)
{
	assert_param(n >= 0 && n <= 3);
	return (n == 0) ? (num & 0xFF) : (one_to_n_bytes(num >> 8, n-1));
}

/**
	* @brief Convert n one-byte variable to an array of n bytes (DECODE)
	* @param n: the number of bytes
	* @param array: the array of n bytes
	*/
s32 n_bytes_to_one(u8* array, u8 n)
{
	assert_param(n >= 1 && n <= 4);
	return (n == 0) ? (array[0] & 0xFF) : ((array[0] & 0xFF) + (n_bytes_to_one(&array[1], n-1) << 8));
}

