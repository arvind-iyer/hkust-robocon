#include "ticks.h"

volatile u16 main_ticks = 0;
volatile u16 seconds = 0;

u16 get_ticks(){
	return main_ticks;
}

u16 get_seconds(){
	return seconds;
}

void ticks_init(void){
	
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	TIM4->ARR = 1000;
	TIM4->PSC = 72 - 1;
	TIM4->EGR = 1;
	TIM4->SR = 0;
	TIM4->DIER = 1;
	TIM4->CR1 = 1;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;   
	NVIC_Init(&NVIC_InitStructure);
}

void TIM4_IRQHandler(void){
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	main_ticks ++;
	if( main_ticks >= 1000 ){
		main_ticks = 0;
		seconds ++;
	}	
}
