#include "configuration.h"
#include "delay.h"


/* variables -------------------------------------------------------------------*/
vu16 ADC_ConvertedValue[160];
vu16 ADC_ConvertedValue2[16];	/*data from core2*/
u8 ADC_ConvertedValue2_raw[32]; /*raw data from core2, data length is 8 bits!!!*/
ErrorStatus HSEStartUpStatus;

u8 calibration_flag_high = 0;
u8 calibration_flag_low = 0;
u8 prev_calibration_flag = 0;


void DMA_Configuration()
{
	DMA_InitTypeDef DMA_InitStructure;
	/* DMA channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 160;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		//内存地址寄存器递增！
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	//Enable DMA channel1
	DMA_Cmd(DMA1_Channel1, ENABLE);

	
	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI2_DR_Base;//(u32)&SPI2->DR;			 //该参数用以定义 DMA外设基地址 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue2_raw;				 //该参数用以定义 DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;								 //	DMA_DIR_PeripheralDST  外设作为数据传输的目的地																					 //DMA_DIR_PeripheralSRC  外设作为数据传输的来源  
	DMA_InitStructure.DMA_BufferSize = 32;									 //DMA_BufferSize 用以定义指定 DMA通道的 DMA缓存的大小，单位为数据单位。根据传输方向，数据单																					 //位等于结构中参数 DMA_PeripheralDataSize 或者参数 DMA_MemoryDataSize 的值。
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					 //DMA_PeripheralInc_Enable  外设地址寄存器递增 																				 //DMA_PeripheralInc_Disable  外设地址寄存器不变 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			 //DMA_PeripheralDataSize设定了外设数据宽度DMA_PeripheralDataSize_Byte  数据宽度为 8 位 ,DMA_PeripheralDataSize_HalfWord  数据宽度为 16 位, DMA_PeripheralDataSize_Word  数据宽度为 32 位 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					 //DMA_MemoryDataSize 设定了外设数据宽度。
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;										 //DMA_Mode设置了 CAN的工作模式,DMA_Mode_Circular  工作在循环缓存模式 ,DMA_Mode_Normal  工作在正常缓存模式 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;							 //DMA_Priority设定 DMA通道SPI_SLAVE_Rx_DMA_Channel的软件优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;										 //DMA_M2M使能 DMA通道的内存到内存传输。
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	
}

void ADC_Configuration()
{
	ADC_InitTypeDef ADC_InitStructure;
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 	//转换由软件而不是外部触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 16;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* Config all the channels --------------------------------------------------*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 9, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 10, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 11, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 12, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 13, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 14, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 15, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 16, ADC_SampleTime_239Cycles5);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/******************************************CALIBRATION******************************************/
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);

	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));
	/************************************************************************************************/

	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void SPI_Configuration(void)
{
    SPI_InitTypeDef SPI_InitStructure;

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_Init(SPI2,&SPI_InitStructure);
	
	DMA_Cmd(DMA1_Channel4, ENABLE);	//Enable DMA
	/* Enable SPI_SLAVE Rx request */
  	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);	//SPI DMA request
	
	SPI_Cmd(SPI2,ENABLE);	//Enable SPI

	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);	//DMA Interrupt config	
}


/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();
	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);
	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus == SUCCESS)
	{
		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1); 		//设置AHB时钟
		/* PCLK2 = HCLK */				  		
		RCC_PCLK2Config(RCC_HCLK_Div1); 		//设置高速AHB时钟
		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);		  	//设置低速AHB时钟
		/* ADCCLK = PCLK2/4 */
		RCC_ADCCLKConfig(RCC_PCLK2_Div4); 	  	
		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);	//设置PLL
		/* Enable PLL */ 
		RCC_PLLCmd(ENABLE);							  			//打开PLL
		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);	  			//设置系统时钟
		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08);
	}

	/* Enable peripheral clocks --------------------------------------------------*/	 //打开要使用的外设时钟
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
	/* Enable USART1 and GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	/* Enable SPI2 as slave */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* This line of code is quite important when doing spi1 remapping */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Enable Internal High Speed oscillator */ 	
	RCC_HSICmd(ENABLE);	   //used in flash access

}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;


	/*SPI2 GPIO configuration*/
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_14;		
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  	GPIO_Init(GPIOB , &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	
	/*Button IO configuration --- commented because of CAN test*/
	/*
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	*/

    /* uart1 tx */ 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			
  	GPIO_Init(GPIOA , &GPIO_InitStructure); 
	
	/* uart1 rx */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;			
  	GPIO_Init(GPIOA , &GPIO_InitStructure); 
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	/*
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/

	/*
	NVIC_InitStructure.NVIC_IRQChannel=DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/
}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configures the USART1.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	/* USART1 configuration ------------------------------------------------------*/
	/* USART1 configured as follow:
		- BaudRate = 9600 baud  
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
		- USART Clock disabled
		- USART CPOL: Clock is active low
		- USART CPHA: Data is captured on the middle 
		- USART LastBit: The clock pulse of the last data bit is not output to 
						 the SCLK pin
	*/
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	
	/* Configure the USART1 synchronous paramters */
	USART_ClockInit(USART1, &USART_ClockInitStructure);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	/* Configure USART1 basic and asynchronous paramters */
	USART_Init(USART1, &USART_InitStructure);
	/* Enable USART1 */
	USART_Cmd(USART1, ENABLE);
}


void USART_OUT16(USART_TypeDef* USARTx, uc16 *Data,uint16_t Len)
{
	uint8_t temp[2];
	uint16_t i;
	for(i=0; i<Len; i++)
	{
		temp[0] = (uint8_t)(Data[i] >> 8);
		temp[1] = (uint8_t)(Data[i] & 0xff);
		USART_OUT(USART1, &temp[0],2);
	}
}

void USART_OUT(USART_TypeDef* USARTx, uc8 *Data,uint16_t Len)
{ 
	uint16_t i;
	for(i=0; i<Len; i++)
	{
		USART_SendData(USARTx, Data[i]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
		//for(tick=0;tick<5000;tick++);
		_delay_ms(2);
	}
}

void USART_TX_BYTE(USART_TypeDef* USARTx, uc8 Data)
{
 	USART_SendData(USARTx, Data);
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
}

 
/*******************************************************************************
* Function Name  : fputc
* Description    : Retargets the C library printf function to the USART or ITM Viewer.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (unsigned char) ch);
	while (!(USART1->SR & USART_FLAG_TXE));
	return (ch);
}





