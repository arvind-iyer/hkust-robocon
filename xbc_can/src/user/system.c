#include "system.h"

void enable_bt_printf(COM_TypeDef COM){
	//uart_init(COM);
	//printf_switch_uart(COM_USART[COM]);
	//uart_interrupt(COM);  
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
