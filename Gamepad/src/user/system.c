#include "system.h"
/*static u8 buzzer_on = 0;
static u16 buzzer_period = 0;
static u8 buzzer_count = 0;
*/

/**
void buzzer_init(void){		   	
	GPIO_InitTypeDef BUZZER_InitStructure; 			  		
	RCC_APB2PeriphClockCmd(BUZZER_RCC, ENABLE);
	BUZZER_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			   
  	BUZZER_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	BUZZER_InitStructure.GPIO_Pin = BUZZER_PIN; 
	GPIO_Init(BUZZER_PORT, &BUZZER_InitStructure); 
}

void buzzer_control(u8 count, u8 period){
	while(count-->0){
		GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
		_delay_ms(period*10);
		GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
		_delay_ms(period*10);
	}	 
}
***/
////////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Initialization of buzzer
  * @param  None
  * @retval None
  */
 /*
void buzzer_init(void)
{		   	
	GPIO_InitTypeDef BUZZER_InitStructure; 	
	NVIC_InitTypeDef NVIC_InitStructure;		  		
	RCC_APB2PeriphClockCmd(BUZZER_RCC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	BUZZER_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			   
	BUZZER_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	BUZZER_InitStructure.GPIO_Pin = BUZZER_PIN; 
	GPIO_Init(BUZZER_PORT, &BUZZER_InitStructure); 
	
	TIM2->ARR = 1000;
	TIM2->PSC = 7199;
	TIM2->EGR = 1;
	TIM2->SR = 0;
	TIM2->DIER = 1;
	TIM2->CR1 = 1;
	TIM_Cmd(TIM2, DISABLE);
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;   
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Timer2 for counting the buzzer
  * @param  None
  * @retval None
  */
  /*
void TIM2_IRQHandler(void)
{
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	
	if (!buzzer_on) {
		GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
		TIM_Cmd(TIM2, DISABLE);
		return;	
	}
	
	if ((buzzer_count--)%2 == 0)
		GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
	else
		GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);

	if (buzzer_count == 0)
		buzzer_on = 0;
}

/**
  * @brief  Generate specific pattern of buzzer
  * @param  count: number of buzz to be generated
  * @param  period: time for each buzz (per 10 ms)
  * @retval None
  */
  /*
void buzzer_control(u8 count, u16 period)
{
	buzzer_count = count*2-1;
	buzzer_on = 1;
	TIM_SetAutoreload(TIM2, period*100-1);
	TIM_SetCounter(TIM2, 0);
	GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
	TIM_Cmd(TIM2, ENABLE);
}
*/
void enable_bt_printf(COM_TypeDef COM){
	uart_init(COM);
	printf_switch_uart(COM_USART[COM]);
	uart_interrupt(COM);  
}

 
u8 _BV(u8 bit) {
	return (1 << bit) ;
}


u16 _BV16(u8 bit) {
	return (1 << bit) ;
}


IRQn_Type interrupChannel[16] = { EXTI0_IRQn , EXTI1_IRQn , EXTI2_IRQn , EXTI3_IRQn , EXTI4_IRQn , EXTI9_5_IRQn , EXTI9_5_IRQn ,EXTI9_5_IRQn ,
					EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn };
					

void external_Interrup_Init( u8 channel , GPIO_TypeDef * GPIOTYPE ){
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	u32 GPIO_RCC , GPIO_SOURCE;
	u16 gpio_pins = 0;
	if( channel >= 16 )
		return;
	gpio_pins = (1 << channel);

//	printf( "channel: %d pin:%d \r\n" , channel , gpio_pins ); 
	
	/* Configure PX.YY pin as input floating */
	GPIO_InitStructure.GPIO_Pin = gpio_pins;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	if( channel == 3 ){
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	}
	GPIO_Init(GPIOTYPE, &GPIO_InitStructure);
	
	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
		
	if( GPIOTYPE == GPIOA ){
		GPIO_RCC = RCC_APB2Periph_GPIOA;
		GPIO_SOURCE = GPIO_PortSourceGPIOA;
	}else if( GPIOTYPE == GPIOB ){
		GPIO_RCC = RCC_APB2Periph_GPIOB;
		GPIO_SOURCE = GPIO_PortSourceGPIOB;
	}else if( GPIOTYPE == GPIOC ){
		GPIO_RCC = RCC_APB2Periph_GPIOC;
		GPIO_SOURCE = GPIO_PortSourceGPIOC;
	}else if( GPIOTYPE == GPIOD ){
		GPIO_RCC = RCC_APB2Periph_GPIOD;
		GPIO_SOURCE = GPIO_PortSourceGPIOD;
	}else if( GPIOTYPE == GPIOE ){
		GPIO_RCC = RCC_APB2Periph_GPIOE;
		GPIO_SOURCE = GPIO_PortSourceGPIOE;
	}else if( GPIOTYPE == GPIOF ){
		GPIO_RCC = RCC_APB2Periph_GPIOF;
		GPIO_SOURCE = GPIO_PortSourceGPIOF;
	}else if( GPIOTYPE == GPIOG ){
		GPIO_RCC = RCC_APB2Periph_GPIOG;
		GPIO_SOURCE = GPIO_PortSourceGPIOG;
	}	
	/* Enable GPIOX clock */
	RCC_APB2PeriphClockCmd(GPIO_RCC, ENABLE);
	
	/* Configure EXTIX line */
	EXTI_InitStructure.EXTI_Line = gpio_pins;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //INT# falling
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearFlag( gpio_pins );
	
	
	GPIO_EXTILineConfig(GPIO_SOURCE, channel);

	//Enable and set EXTIX Interrupt to the lowest priority 
	NVIC_InitStructure.NVIC_IRQChannel = interrupChannel[channel] ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
