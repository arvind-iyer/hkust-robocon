#include "Robocon_CAN.h"

CanTxMsg TxMessage;
CanRxMsg RxMessage;
CanRxMsg CAN_Queue[CAN_QUEUE_LENGTH];	//CAN Msg buffer
s32 CAN_QueueCounter = 0;					//number of entries in the queue
u8 CAN_QueueHead = 0;
u8 CAN_QueueTail = 0;
CanRxMsg CAN_queueHead;
u8 can_rx_data = 0;
//extern u8 sys_counter;

/* Devices' data ---------------------------*/
u8 Device1_data = 0;
u8 Device2_data = 0;
u8 Device3_data = 0;
u8 Device4_data = 0;

void CAN_GPIO_Configuration(void)
{
/*
	GPIO_InitTypeDef GPIO_InitStructure;
			
	// Configure CAN pin: RX 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Configure CAN pin: TX 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
*/
	GPIO_InitTypeDef GPIO_InitStructure;
			
	/* Configure CAN pin: RX */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	*/
	/* Configure CAN pin: TX */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	*/

	//GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
	//GPIO_PinRemapConfig(GPIO_Remap2_CAN1, ENABLE);
}

void CAN_NVIC_Configuration(void)
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

	/* enabling interrupt */
	NVIC_InitStructure.NVIC_IRQChannel=USB_LP_CAN1_RX0_IRQn; //USB低优先级或者CAN接收0中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
void CAN_Configuration(void)
{
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
	CAN_GPIO_Configuration();
	CAN_NVIC_Configuration();

	/* GPIO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	
	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	
	/* CAN Baudrate = 1mbps*/
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 6;
	CAN_Init(CAN1, &CAN_InitStructure);

	/* CAN FIFO0 message pending interrupt enable */ 
  	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	
	/* CAN filter init */
	#ifdef MAIN_CONTROL
	/* 0xA0~0xAF */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0A0 << 5;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFE1F;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	/* 0xB0~0xBF */
	CAN_FilterInitStructure.CAN_FilterNumber = 1;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0B0 << 5;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFE1F;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	/* 0xC0~0xCF */
	CAN_FilterInitStructure.CAN_FilterNumber = 2;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0C0 << 5;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFE1F;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	#else
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = MOTOR_ID << 5;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	#endif

}

void Main_TX(void)
{
	u8 retry = 100;
	u8 TX_MailBox = 0;
	TxMessage.StdId = 0x321;
	TxMessage.ExtId = 0x00;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 1;

	//TxMessage.Data[0] = sys_counter;
	TX_MailBox = CAN_Transmit(CAN1, &TxMessage);
	//while(CAN_TransmitStatus(CAN1,TX_MailBox) != CANTXOK && retry--);
	//TxMessage.StdId = 0x322;
	//while(!checkMailBoxOK());
	//TX_MailBox = CAN_Transmit(CAN1, &TxMessage);
	//while(CAN_TransmitStatus(CAN1,TX_MailBox) != CANTXOK && retry--);
	
	//TxMessage.StdId = 0x323;
	//TX_MailBox = CAN_Transmit(CAN1, &TxMessage);
	//while(CAN_TransmitStatus(CAN1,TX_MailBox) != CANTXOK && retry--);
	/*
	TxMessage.StdId = 0x324;
	while(!checkMailBoxOK());
	TX_MailBox = CAN_Transmit(CAN1, &TxMessage);
	//while(CAN_TransmitStatus(CAN1,TX_MailBox) != CANTXOK);
	*/
}

void Device1_TX(void)
{
	/* Transmit */
	TxMessage.StdId = 0x321;
	TxMessage.ExtId = 0x00;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 1;

	TxMessage.Data[0] = 0xFF;
	//TxMessage.Data[1] = 0xBB;
	CAN_Transmit(CAN1, &TxMessage);
}

void Device2_TX(void)
{
	/* Transmit */
	TxMessage.StdId = 0x322;
	TxMessage.ExtId = 0x00;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 1;

	TxMessage.Data[0] = 0xBB;
	//TxMessage.Data[1] = 0xBB;
	CAN_Transmit(CAN1, &TxMessage);
}

void Device3_TX(void)
{
	/* Transmit */
	TxMessage.StdId = 0x323;
	TxMessage.ExtId = 0x00;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 1;

	TxMessage.Data[0] = 0xCC;
	//TxMessage.Data[1] = 0xBB;
	CAN_Transmit(CAN1, &TxMessage);
}

void Device4_TX(void)
{
	/* Transmit */
	TxMessage.StdId = 0x324;
	TxMessage.ExtId = 0x00;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 1;

	TxMessage.Data[0] = 0xDD;
	//TxMessage.Data[1] = 0xBB;
	CAN_Transmit(CAN1, &TxMessage);
}

/* xie de tai shi****************************************************************/ 
/*
void CAN_addToQueue(CanRxMsg RxMsg)
{										   	
	CAN_Queue[queueCounter] = RxMsg;			//add the new message to the queue
	queueCounter++;								//update the queue counter			
}

u8 CAN_dequeue(void)
{
	u8 i;
	CanRxMsg firstEntry;
	if(queueCounter != 0)
	{
		firstEntry = CAN_Queue[0];
		for(i = 1; i < queueCounter-1; i++)		//start from the second entry
		{								  
			CAN_Queue[i-1] = CAN_Queue[i];
		}
		queueCounter--;							//update the queue counter
		CAN_queueHead = firstEntry;				//return the first entry
		return 1;
	}
	else
		return CAN_QUEUE_EMPTY;		
}
*/
void CAN_addToQueue(CanRxMsg RxMsg)
{										   	
	CAN_Queue[CAN_QueueTail] = RxMsg;			//add the new message to the queue
	CAN_QueueTail++;							//update the queue counter
	if(CAN_QueueTail == CAN_QUEUE_LENGTH)		//circular
		CAN_QueueTail = 0;
	CAN_QueueCounter++;
}

u8 CAN_dequeue(void)
{
	if(CAN_QueueCounter != 0)
	{
		CAN_queueHead = CAN_Queue[CAN_QueueHead];
		CAN_QueueHead++;
		if(CAN_QueueHead == CAN_QUEUE_LENGTH)
			CAN_QueueHead = 0;
		CAN_QueueCounter--;
		return 1;
	}
	else
		return CAN_QUEUE_EMPTY;			
}

void CAN_messageProcessing(CanRxMsg RxMsg)
{
	switch(RxMsg.StdId)	 						//each device has a unique id
	{
		case DEVICE_ONE:
			Device1_data = RxMsg.Data[0];
			break;					   

		case DEVICE_TWO:
			Device2_data = RxMsg.Data[0];
			break;
			
		case DEVICE_THREE:
		  	Device3_data = RxMsg.Data[0];
			break;
			
		case DEVICE_FOUR:
			Device4_data = RxMsg.Data[0];
			break;
		
		default:
			break; 
	}												
}										  	

