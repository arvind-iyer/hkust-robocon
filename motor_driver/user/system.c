/**
  ******************************************************************************
  * @file    system.c
  * @author  William LEE
  * @version V3.5.0
  * @date    24-January-2015
  * @brief   This file provides systicks and system clocks information
  ******************************************************************************
  * @attention
  *
  * These functions are inheritance from Paul GAO, USART3 is no longer used,
	* but still keep here for backup. To be deleted to improve independence
	*
  ******************************************************************************
  */#include "system.h"

void SetupClock (void){
	ErrorStatus HSEStartUpStatus;	/* RCC system reset(for debug purpose) */
	RCC_DeInit();									/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON); 		/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS) {
	   FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);  			/* Enable Prefetch Buffer */
	   FLASH_SetLatency(FLASH_Latency_2);		   											/* Flash 2 wait state */
	   RCC_HCLKConfig(RCC_SYSCLK_Div1);															/* HCLK = SYSCLK  */
	   RCC_PCLK2Config(RCC_HCLK_Div1);															/* PCLK2 = HCLK , high speed AHB2=72MHz*/
	   RCC_PCLK1Config(RCC_HCLK_Div2);															/* PCLK1 = HCLK/2  low speed AHB1=36MHz */
	   RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	   RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); 				//	  ((uint32_t)0x00010000)
																																	/* PLLCLK = 8MHz * 9 = 72 MHz */
	   RCC_PLLCmd(ENABLE);																					/* Enable PLL */
	   while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);					/* Wait till PLL is ready */
	   RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);										/* Select PLL as system clock source */
	   while (RCC_GetSYSCLKSource() != 0x08);												/* Wait till PLL is used as system clock source */
	}
}

void NVIC_Configuration(void){ 
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}

void USART3_Configuration(void){
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USART2 Tx (PC_10) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* Configure USART2 Rx (PC_11) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3,ENABLE);
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	/* Configure the USART3 synchronous paramters */
	USART_ClockInit(USART3, &USART_ClockInitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* Configure USART1 basic and asynchronous paramters */
	USART_Init(USART3, &USART_InitStructure);
	
	/* Enable USART1 Receive and Transmit interrupts */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
//	USART_ITConfig(USART3, USART_IT_TC, ENABLE);
	/* Enable USART1 */
	USART_Cmd(USART3, ENABLE);
}

void SysTick_Configuration(u16 freq){
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	SysTick_Config(72000000 / freq);
}

void system_init(u16 sysFreq) //in Hz
{
	SetupClock();
	USART3_Configuration();
	SysTick_Configuration(sysFreq);	
	NVIC_Configuration(); 
}
