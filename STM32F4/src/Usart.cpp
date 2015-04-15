/*
 * Usart.cpp
 *
 *  Created on: 2014�~8��4��
 *      Author: YunKei
 */

#include <Usart.h>
#include <stm32f4xx_usart.h>
#include <stdio.h>
#include <Leds.h>
#include <stm32f4xx_it.h>
#include <Ticks.h>
#include <stdarg.h>
#include <stm32f4xx_dma.h>
//#include <Delay.h>
//#include <Task.h>

extern USART_TypeDef* STDOUT_USART;
extern USART_TypeDef* STDERR_USART;
extern USART_TypeDef* STDIN_USART;

Usart* _mUsart1;
Usart* _mUsart3;
Usart* _mUart4;
Usart* _mUart5;

void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) == SET)
	{
		DMA_ClearITPendingBit (DMA2_Stream7, DMA_IT_TCIF7);
		DMA_Cmd(DMA2_Stream7, DISABLE);
		Ticks::getInstance()->setTimeout(3);
		while (USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET){
			if(Ticks::getInstance()->Timeout()){
				break;
			}
		}
		USART_ClearFlag(USART1,USART_FLAG_TC);
		Usart::getInstance(USART1)->setIsDmaBusy(false);
	}
}

void DMA1_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) == SET)
	{
		DMA_ClearITPendingBit (DMA1_Stream3, DMA_IT_TCIF3);
		DMA_Cmd(DMA1_Stream3, DISABLE);
		Ticks::getInstance()->setTimeout(3);
		while (USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET){
			if(Ticks::getInstance()->Timeout()){
				break;
			}
		}
		USART_ClearFlag(USART3,USART_FLAG_TC);
		Usart::getInstance(USART3)->setIsDmaBusy(false);
	}
}

void DMA1_Stream1_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1) == SET)
	{
		DMA_ClearITPendingBit (DMA1_Stream1, DMA_IT_TCIF1);
		DMA_Cmd(DMA1_Stream1, DISABLE);
		Ticks::getInstance()->setTimeout(3);
		while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET){
			if(Ticks::getInstance()->Timeout()){
				break;
			}
		}
		for(int i = 0; i < 8; i++){
			Usart::getInstance(USART3)->getBuffer()[Usart::getInstance(USART3)->getBufferCount()] = Usart::getInstance(USART3)->getRxBuffer()[i];
			Usart::getInstance(USART3)->setBufferCount(Usart::getInstance(USART3)->getBufferCount() + 1);
			if(Usart::getInstance(USART3)->getBufferCount() == 2047){
				Usart::getInstance(USART3)->setBufferCount(0);
			}
		}

		DMA_Cmd(DMA1_Stream1, ENABLE);
	}
}

void DMA2_Stream2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) == SET)
	{
		DMA_ClearITPendingBit (DMA2_Stream2, DMA_IT_TCIF2);
		DMA_Cmd(DMA2_Stream2, DISABLE);
		Ticks::getInstance()->setTimeout(3);
		while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET){
			if(Ticks::getInstance()->Timeout()){
				break;
			}
		}
		for(int i = 0; i < 8; i++){
			Usart::getInstance(USART1)->getBuffer()[Usart::getInstance(USART1)->getBufferCount()] = Usart::getInstance(USART1)->getRxBuffer()[i];
			Usart::getInstance(USART1)->setBufferCount(Usart::getInstance(USART1)->getBufferCount() + 1);
			if(Usart::getInstance(USART1)->getBufferCount() == 2047){
				Usart::getInstance(USART1)->setBufferCount(0);
			}
		}
		DMA_Cmd(DMA2_Stream2, ENABLE);
	}
}


void USART1_IRQHandler(){

	Usart::getInstance(USART1)->getBuffer()[Usart::getInstance(USART1)->getBufferCount()] = USART_ReceiveData(USART1);
	Usart::getInstance(USART1)->setBufferCount(Usart::getInstance(USART1)->getBufferCount() + 1);
	if(Usart::getInstance(USART1)->getBufferCount() == 2047){
		Usart::getInstance(USART1)->setBufferCount(0);
	}
}

void USART3_IRQHandler(){

	Usart::getInstance(USART3)->getBuffer()[Usart::getInstance(USART3)->getBufferCount()] = USART_ReceiveData(USART3);
	Usart::getInstance(USART3)->setBufferCount(Usart::getInstance(USART3)->getBufferCount() + 1);
	if(Usart::getInstance(USART3)->getBufferCount() == 2047){
		Usart::getInstance(USART3)->setBufferCount(0);
	}
}

void UART4_IRQHandler(){

	Usart::getInstance(UART4)->getBuffer()[Usart::getInstance(UART4)->getBufferCount()] = USART_ReceiveData(UART4);
	Usart::getInstance(UART4)->setBufferCount(Usart::getInstance(UART4)->getBufferCount() + 1);
	if(Usart::getInstance(UART4)->getBufferCount() == 2047){
		Usart::getInstance(UART4)->setBufferCount(0);
	}
}

void UART5_IRQHandler(){

	Usart::getInstance(UART5)->getBuffer()[Usart::getInstance(UART5)->getBufferCount()] = USART_ReceiveData(UART5);
	Usart::getInstance(UART5)->setBufferCount(Usart::getInstance(UART5)->getBufferCount() + 1);
	if(Usart::getInstance(UART5)->getBufferCount() == 2047){
		Usart::getInstance(UART5)->setBufferCount(0);
	}
}

bool Usart::getIsDmaBusy(){
	return isDmaBusy;
}

void Usart::setIsDmaBusy(bool value){
	isDmaBusy = value;
}

void Usart::setBufferCount(int value){
	BufferCount = value;
}

int Usart::getBufferCount(){
	return BufferCount;
}

char* Usart::getBuffer(){
	return pBuffer;
}

char* Usart::getRxBuffer(){
	return rxBuffer;
}

int Usart::Read(unsigned char* buffer, int length){

	for(int i = 0; i < length; i++){
		if(pBuffer >= Buffer + 2047){
			pBuffer = Buffer;
		}
		buffer[i] = *(pBuffer++);
	}
	buffer[length] = '\0';
	BufferCount -= length;
	return BufferCount;
}

Usart* Usart::getInstance(USART_TypeDef* UARTx){
	Usart* usart = 0;
	if (UARTx == USART1){
		usart = _mUsart1;
	}
	else if (UARTx == USART3){
		usart = _mUsart3;
	}
	else if (UARTx == UART4){
		usart = _mUart4;
	}
	else if (UARTx == UART5){
		usart = _mUart5;
	}
	return usart;
}

void Usart::setPrintUsart(USART_TypeDef* UARTx){
	STDOUT_USART = UARTx;
	STDERR_USART = UARTx;
	STDIN_USART = UARTx;
}

int usartDelayCount;

void resetTask1(){
	if(usartDelayCount++ > 10){
		usartDelayCount = 0;
		Usart(USART1, Usart::getInstance(USART1)->getBaudrate(), true);
//		Task::getInstance()->DeAttach(resetTask1);
	}
}

void resetTask3(){
	if(usartDelayCount++ > 10){
		usartDelayCount = 0;
		Usart(USART3, Usart::getInstance(USART3)->getBaudrate(), true);
//		Task::getInstance()->DeAttach(resetTask3);
	}
}

void resetTask4(){
	if(usartDelayCount++ > 10){
		usartDelayCount = 0;
		Usart(UART4, Usart::getInstance(UART4)->getBaudrate(), true);
//		Task::getInstance()->DeAttach(resetTask4);
	}
}

void resetTask5(){
	if(usartDelayCount++ > 10){
		usartDelayCount = 0;
		Usart(UART5, Usart::getInstance(UART5)->getBaudrate(), true);
//		Task::getInstance()->DeAttach(resetTask5);
	}
}

void Usart::reset(){
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	usartDelayCount = 0;
	if (_Usart == USART1)
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
//		Task::getInstance()->Attach(10, 0, resetTask1, true, -1);
	}
	else if(_Usart == USART3){
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
//		Task::getInstance()->Attach(10, 0, resetTask3, true, -1);

	}
	else if (_Usart == UART4)
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
//		Task::getInstance()->Attach(10, 0, resetTask4, true, -1);
	}
	else if (_Usart == UART5)
	{
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
//		Task::getInstance()->Attach(10, 0, resetTask5, true, -1);
	}
}

Usart::Usart(USART_TypeDef* UARTx, uint32_t baudrate, bool createdInstance) : _baudrate(baudrate), BufferCount(0), pBuffer(Buffer), isDmaBusy(false){

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	USART_InitStructure.USART_BaudRate = _baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	if (UARTx == USART1)
	{
		if(!createdInstance){
			_mUsart1 = this;
		}
		USART_DeInit(USART1);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		USART_Init(USART1, &USART_InitStructure);
		USART_Cmd(USART1, ENABLE);
		//NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;

		RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_DMA2, ENABLE);
		DMA_DeInit(DMA2_Stream7);

		DMA_InitStructure.DMA_BufferSize = 32;
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
		DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)(txBuffer) ;
		DMA_Init(DMA2_Stream7, &DMA_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init (&NVIC_InitStructure);

		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
		DMA_ITConfig (DMA2_Stream7, DMA_IT_TC, ENABLE);


		DMA_DeInit(DMA2_Stream2);

		DMA_InitStructure.DMA_BufferSize = 8;
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
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
		DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)(rxBuffer) ;
		DMA_Init(DMA2_Stream2, &DMA_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init (&NVIC_InitStructure);

		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
		DMA_ITConfig (DMA2_Stream2, DMA_IT_TC, ENABLE);
		DMA_Cmd(DMA2_Stream2, ENABLE);
	}
	else if(UARTx == USART3){

		if(!createdInstance){
			_mUsart3 = this;
		}
		USART_DeInit(USART3);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		USART_Init(USART3, &USART_InitStructure);

//		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
//		NVIC_EnableIRQ(USART3_IRQn);

		USART_Cmd(USART3, ENABLE);

		RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_DMA1, ENABLE);
		DMA_DeInit(DMA1_Stream3);

		DMA_InitStructure.DMA_BufferSize = 32;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART3->DR)) ;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;

		DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
		DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)(txBuffer) ;
		DMA_Init(DMA1_Stream3, &DMA_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init (&NVIC_InitStructure);

		USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
		DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);


		DMA_DeInit(DMA1_Stream1);

		DMA_InitStructure.DMA_BufferSize = 8;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART3->DR)) ;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;

		DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
		DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)(rxBuffer) ;
		DMA_Init(DMA1_Stream1, &DMA_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init (&NVIC_InitStructure);

		USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
		DMA_ITConfig (DMA1_Stream1, DMA_IT_TC, ENABLE);
		DMA_Cmd(DMA1_Stream1, ENABLE);

	}
	else if (UARTx == UART4)
	{
		if(!createdInstance){
			_mUart4 = this;
		}
		USART_DeInit(UART4);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		USART_Init(UART4, &USART_InitStructure);
		USART_Cmd(UART4, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	}
	else if (UARTx == UART5)
	{
		if(!createdInstance){
			_mUart5 = this;
		}
		USART_DeInit(UART5);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
		USART_Init(UART5, &USART_InitStructure);
		USART_Cmd(UART5, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	}

	//USART_ITConfig(UARTx, USART_IT_RXNE, ENABLE);
	//NVIC_Init(&NVIC_InitStructure);
	setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    _Usart = UARTx;
}

uint32_t Usart::getBaudrate(){
	return _baudrate;
}

void Usart::Print(const char* pstr, ...)
{
	int length = 0;
	va_list arglist;
	char* fp;
	for(int i = 0; i < 64; i++){
		txBuffer[i] = 0;
	}
	va_start(arglist, pstr);
	vsprintf(txBuffer, pstr, arglist);
	va_end(arglist);

	fp = txBuffer;

	while(*(fp++)){
		length++;
	}

	if(_Usart == USART1){
		if(!Usart::getInstance(USART1)->getIsDmaBusy()){
			Usart::getInstance(USART1)->setIsDmaBusy(true);
			DMA_SetCurrDataCounter(DMA2_Stream7, length);
			DMA_Cmd(DMA2_Stream7, ENABLE);
		}
	}
	else if(_Usart == USART3){
		if(!Usart::getInstance(USART3)->getIsDmaBusy()){
			Usart::getInstance(USART3)->setIsDmaBusy(true);
			DMA_SetCurrDataCounter(DMA1_Stream3, length);
			DMA_Cmd(DMA1_Stream3, ENABLE);
		}
	}

//	while(*fp){
//		Ticks::getInstance()->setTimeout(3);
//		while (USART_GetFlagStatus(_Usart, USART_FLAG_TXE) == RESET){
//			if(Ticks::getInstance()->Timeout()){
//				return;
//			}
//		}
//		USART_SendData(_Usart, *(fp++));
//	}
}
