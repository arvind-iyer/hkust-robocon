/**
  ******************************************************************************
  * @file    usart.c
  * @author  Cheung Ngai{JOHN}
  * @version V1.0.0
  * @date    17-AUG-2015
  * @brief   This file provide you a USART with DMA. It helps you to read USART signal 
  *				and send USART signal
  ******************************************************************************
  * @attention
  *
  * This source is designed for application use. Unless necessary, try NOT to
	* modify the function definition. The constants which are more likely to 
	* vary among different schematics have been placed as pre-defined constant
	* (i.e., "#define") in the header file.
	*
  ******************************************************************************
  */
	
#include "usart.h"	
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "1.8 tft_display.h"

char txBuffer[64];
char rxBuffer[8];

void uart_init(u32 baudrate){
   
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_DMA2, ENABLE);
	
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART1,&USART_InitStructure);
    USART_Cmd(USART1, ENABLE);  
	
/*dma tx init*/

		DMA_DeInit(DMA2_Stream7);

		DMA_InitStructure.DMA_BufferSize =32;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART1->DR)) ;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
		DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)(txBuffer);
		
		DMA_Init(DMA2_Stream7, &DMA_InitStructure);

        USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
		

		/* DMA RX INIT
		DMA2 STREAM2 CHANNEL4
		*/
		

		DMA_DeInit(DMA2_Stream2);

		DMA_InitStructure.DMA_BufferSize = 8;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//will always receive data but just put there not do anything
		DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART1->DR)) ;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;

		DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;//rx
		DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)(rxBuffer) ;
		DMA_Init(DMA2_Stream2, &DMA_InitStructure);

		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

		DMA_Cmd(DMA2_Stream2, ENABLE);

}


static bool USART_FIRST_TX =true;
void usart_tx_one_byte(char data) 		//only send one data when call this function once
{

    if(SET == DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)|| (USART_FIRST_TX==true))  //TCIF = 1 = SET A transfer complete event happen 0= no transfer complete event
    {  USART_FIRST_TX=false;
       DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);
		
        DMA_Cmd(DMA2_Stream7,DISABLE);
        txBuffer[0] = data;
		
		DMA_SetCurrDataCounter(DMA2_Stream7, strlen(&data));
        DMA_Cmd(DMA2_Stream7,ENABLE);
         
    }
	
}	

void usart_print(const char* pstr, ...){				
	
	if((SET == DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)) || (USART_FIRST_TX==true)){
		USART_FIRST_TX=false;
		
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);
	DMA_Cmd(DMA2_Stream7, DISABLE);
		
	int length = 0;
	va_list arglist;
	char* fp;
	for(int i = 0; i <64; i++){  
		txBuffer[i] = 0;
	}
	va_start(arglist, pstr);
	vsprintf(txBuffer, pstr, arglist);
	va_end(arglist);

	fp = txBuffer;

	while(*(fp++)){
		length++;
	}
	DMA_SetCurrDataCounter(DMA2_Stream7, length);
	DMA_Cmd(DMA2_Stream7, ENABLE);
	}
}




void usart_rx(){

	
	
	tft_prints(0,6,":%c",rxBuffer[0]);
	tft_prints(3,6,":%c",rxBuffer[1]);
	tft_prints(6,6,":%c",rxBuffer[2]);
	tft_prints(0,7,":%c",rxBuffer[3]);
	tft_prints(3,7,":%c",rxBuffer[4]);
	tft_prints(6,7,":%c",rxBuffer[5]);
	tft_prints(0,8,":%c",rxBuffer[6]);
	tft_prints(3,8,":%c",rxBuffer[7]);
	
	tft_prints(0,9,":%s",rxBuffer);
	
	if (!strcmp(rxBuffer,"12345678"))
	{
	
	tft_prints(6,8,":same");
	}	
	int data_count=DMA_GetCurrDataCounter(DMA2_Stream2);
	tft_prints(0,5,"%d",data_count);
	
	if(data_count<8){
	
		for(int a=data_count;a<=8;++a)
		{rxBuffer[data_count+1]=0;}
	
	}
}


char *usart_rx_return_all_buffer()	//this function is for returning 8 character rx buffer
{
return rxBuffer;
}


char usart_rx_return_1_byte(int buffer_number)    //this function is for returning 1 character in specific rx buffer
{
return rxBuffer[buffer_number];
}

