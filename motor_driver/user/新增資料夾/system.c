#include "system.h"

USART_InitTypeDef USART_InitStructure;
USART_ClockInitTypeDef  USART_ClockInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

u16 CCR1_Val = 0;
u16 CCR2_Val = 0;
u16 CCR3_Val = 0;
u16 CCR4_Val = 0;
u16 PrescalerValue = 0;

u8 connect_state = 0;
u8 control_state = 0;

extern u8 last_motor1;
extern u8 last_motor2;
extern u8 last_servo[6];



void SetupClock (void){
	ErrorStatus HSEStartUpStatus;	/* RCC system reset(for debug purpose) */
	RCC_DeInit();	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON); /* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS)
	{
	   FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);  		/* Enable Prefetch Buffer */
	   FLASH_SetLatency(FLASH_Latency_2);		   					/* Flash 2 wait state */
	   RCC_HCLKConfig(RCC_SYSCLK_Div1);								/* HCLK = SYSCLK  */
	   RCC_PCLK2Config(RCC_HCLK_Div1);								/* PCLK2 = HCLK , high speed AHB2=72MHz*/
	   RCC_PCLK1Config(RCC_HCLK_Div2);								/* PCLK1 = HCLK/2  low speed AHB1=36MHz */
	   RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	   RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); //	  ((uint32_t)0x00010000)
	   																/* PLLCLK = 8MHz * 9 = 72 MHz */
	   RCC_PLLCmd(ENABLE);											/* Enable PLL */
	   while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);			/* Wait till PLL is ready */
	   RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);					/* Select PLL as system clock source */
	   while (RCC_GetSYSCLKSource() != 0x08);						/* Wait till PLL is used as system clock source */
	}
}

void RCC_Configuration(void){
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|
  						 RCC_APB1Periph_TIM3|
						 RCC_APB1Periph_TIM4|
						 RCC_APB1Periph_TIM5|
						 RCC_APB1Periph_USART3 , ENABLE);

  /* GPIOC clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|
  						 RCC_APB2Periph_GPIOB|
 						 RCC_APB2Periph_GPIOC|
						 RCC_APB2Periph_GPIOD|
						 RCC_APB2Periph_AFIO |	  //alternate function clock
						 //RCC_APB2Periph_USART1|
						 RCC_APB2Periph_TIM1|
						 RCC_APB2Periph_TIM8,ENABLE);
}


void GPIO_init(void){
/*
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
*/

  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void timer3_pwm_init(void){

   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   TIM_OCInitTypeDef  TIM_OCInitStructure;
   //GPIO_InitTypeDef GPIO_InitStructure;
/* GPIOA Configuration:TIM1 ch1 2 3 4, output pp*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 1799;  // 40KHZ    4500;	  //16KHz
  TIM_TimeBaseStructure.TIM_ClockDivision =  TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);


  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);
  TIM_CtrlPWMOutputs(TIM3, ENABLE);

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
	SysTick_Config(SystemCoreClock / freq);
}
/*
void system_shake_hand (u8 mode)
{
	connect_state = mode;
	USART_SendData(USART3, mode);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	USART_SendData(USART3, stop_byte1);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	USART_SendData(USART3, stop_byte2);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}
*/
void Encoder_init1(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef encoder_TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	
	/* GPIOA Configuration: TIM5 ch1, input float*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* GPIOB Configuration:TIM5 ch2, input float*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 


	/* Timer configuration in Encoder mode */
	encoder_TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling
	encoder_TIM_TimeBaseStructure.TIM_Period = 65535;
	encoder_TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	encoder_TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM2, &encoder_TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,//TIM_EncoderMode_TI12->count on 4 edge per cycle
	                         TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 8;//ICx_FILTER:1000：采样频率fSAMPLING=fDTS/8，N=6;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	// Clear all pending interrupts
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM2->CNT = 32768;
	
	TIM_Cmd(TIM2, ENABLE);
}
vu16 ADC_ConvertedValue[2];

void _ADC_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 	/* Configure PC.04 (ADC Channel14) as analog input -------------------------*/
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
}

void _DMA_Configuration(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void _DMA_Initialization(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 2;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		//内存地址寄存器递增！
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel1, ENABLE);
}

void _ADC_Initialization(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 	//转换由软件而不是外部触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 2;
    ADC_Init(ADC1, &ADC_InitStructure);
	  
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5);	   //12M/(239.5+12.5)=47.62 KHz
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_239Cycles5);	
    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);
	  
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
}

void ADC_Calibaration(void)
{
    /* Enable ADC1 reset calibaration register */   
    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* Start ADC1 calibaration */
    ADC_StartCalibration(ADC1);

    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}



void system_init(u16 sysFreq) //in Hz
{
	SetupClock();
	RCC_Configuration();

	GPIO_init();

	timer3_pwm_init();
	
	USART3_Configuration();

	Encoder_init1();
										
	SysTick_Configuration(sysFreq);
	
	NVIC_Configuration();
  
	connect_state = 0;

	_ADC_Configuration();

	_DMA_Configuration();

	_DMA_Initialization();

    _ADC_Initialization();

    ADC_Calibaration();
 
}

void system_pwm_enable(void)
{
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void system_para_init(void)
{
 	motion_set_motor(0 , 1);

	connect_state = 0;
}
