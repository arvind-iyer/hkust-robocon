#include "ticks.h"

static volatile u16 main_ticks = 0;
static volatile u16 seconds = 0;

u16 get_ticks(){
	return main_ticks;
}

u16 get_seconds(){
	return seconds;
}

void ticks_init(void){
	
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   	// TimeBase is for timer setting 
	RCC_APB1PeriphClockCmd(TICKS_RCC , ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 1000;	                 				       // Timer period, 1000 ticks in one second
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1;     // 72M/1M - 1 = 71
	TIM_TimeBaseInit(TICKS_TIM, &TIM_TimeBaseStructure);      							 // this part feeds the parameter we set above
	
	TIM_ClearITPendingBit(TICKS_TIM, TIM_IT_Update);												 // Clear Interrupt bits
	TIM_ITConfig(TICKS_TIM, TIM_IT_Update, ENABLE);													 // Enable TIM Interrupt
	TIM_Cmd(TICKS_TIM, ENABLE);																							 // Counter Enable

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TICKS_IRQn;   
	NVIC_Init(&NVIC_InitStructure);
}

TICKS_IRQHandler
{
	TIM_ClearFlag(TICKS_TIM, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TICKS_TIM, TIM_IT_Update);
	++main_ticks;
	if( main_ticks >= 1000 ){
		main_ticks = 0;
		++seconds;
	}	
}
