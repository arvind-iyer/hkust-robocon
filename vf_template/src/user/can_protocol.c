/**********************************************************************
*				HKUST ROBOCON 2014 CAN PROTOCOL						
*		
*Author: Joey Bi
*Date: JAN, 2014		
*Description: This file defines the CAN communication protocol
			  between the main control and the sub-systems.
				!! Developed from 2013 CAN PROTOCOL by ZHANG LIANG GUANG !!
*Updates: 
*			The New Sensor Board CAN Complete     --    JAN 5, 2014
*			New CAN Protocol to transmit SENSOR signal -- JAN 6, 2014

***********************************************************************/

#include "can_protocol.h"

/* Private variables -----------------------------------------*/

CanTxMsg Tx_Queue[TX_QUEUE_LENGTH];
u8 Tx_QueueHead = 0;
u8 Tx_QueueTail = 0;
u8 Tx_QueueCounter = 0;
CanRxMsg Rx_Queue[RX_QUEUE_LENGTH];
u8 Rx_QueueHead = 0;
u8 Rx_QueueTail = 0;
u8 Rx_QueueCounter = 0;



CanTxMsg General_Encoding(u32 Device_ID, Data_Field Cmd_Data)
{
	u8 i;
	CanTxMsg TxMessage; 
	TxMessage.StdId = Device_ID;
	TxMessage.ExtId = 0x00;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = Cmd_Data.Data_Length;
	for(i = 0; i < Cmd_Data.Data_Length; i++)
	{
		TxMessage.Data[i] = Cmd_Data.Data[i];		
	}
	return TxMessage;
}


/* CAN Tx message add to queue ------------------------------------------*/
void CAN_Tx_addToQueue(CanTxMsg TxMessage)
{
	Tx_Queue[Tx_QueueTail] = TxMessage;
	Tx_QueueTail++;
	if(Tx_QueueTail == TX_QUEUE_LENGTH)
		Tx_QueueTail = 0;
	Tx_QueueCounter++;
}

/* CAN Tx message dequeue and transmission -------------------------------*/
u8 CAN_Tx_dequeue(void)
{
	CanTxMsg TxMsg;
	u8 Tx_MailBox = 0;
	u16 retry = RETRY_TIMEOUT;
	if(Tx_QueueCounter != 0)
	{
		TxMsg = Tx_Queue[Tx_QueueHead];
		Tx_QueueHead++;
		if(Tx_QueueHead == TX_QUEUE_LENGTH)
			Tx_QueueHead = 0;
		Tx_QueueCounter--;
		Tx_MailBox = CAN_Transmit(CAN1, &TxMsg);							//transmit the message
		while(CAN_TransmitStatus(CAN1,Tx_MailBox) != CANTXOK && retry--);	//wait for the transmission to be finished
		return 1;
	}
	else
		return CAN_QUEUE_EMPTY;			
}


void CAN_Rx_addToQueue(CanRxMsg RxMessage)
{										   	
	Rx_Queue[Rx_QueueTail] = RxMessage;
	Rx_QueueTail++;
	if(Rx_QueueTail == RX_QUEUE_LENGTH)
		Rx_QueueTail = 0;
	Rx_QueueCounter++;
}

/* CAN Rx message processing ---------------------------------------------*/
void CAN_Rx_Processing(CanRxMsg RxMessage)
{		
} 

/* CAN Rx message dequeue and processing ---------------------------------*/
u8 CAN_Rx_dequeue(void)
{
	CanRxMsg RxMsg;
	if(Rx_QueueCounter != 0)
	{
		RxMsg = Rx_Queue[Rx_QueueHead];
		Rx_QueueHead++;
		if(Rx_QueueHead == RX_QUEUE_LENGTH)
			Rx_QueueHead = 0;
		Rx_QueueCounter--;
		CAN_Rx_Processing(RxMsg);
		return 1;
	}
	else
		return CAN_QUEUE_EMPTY;			
}

void CAN_Tx_update(void)
{
	while(CAN_Tx_dequeue() != CAN_QUEUE_EMPTY);
}

void CAN_Rx_update(void)
{
	while(CAN_Rx_dequeue() != CAN_QUEUE_EMPTY);
}

/*Interrupt is handled here*/

void USB_LP_CAN_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	//CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

	if(RxMessage.IDE == CAN_ID_STD)
	CAN_Rx_Processing(RxMessage);
}



void can_ticks_test(u16 ticks, u16 seconds) 
{
	Data_Field data;
	CanTxMsg TxMsg;
	data.Data_Length = 2;
	data.Data[0] = (u8) (ticks & 0xFF);								//indicates the type of cmd
	data.Data[1] = (u8) (seconds & 0xFF);
	TxMsg = General_Encoding(0xA0, data);
	CAN_Tx_addToQueue(TxMsg);
}



