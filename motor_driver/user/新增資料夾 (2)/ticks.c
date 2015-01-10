#include "ticks.h"

volatile u16 ticks = 0;
volatile u16 seconds = 0;

u16 get_ticks(){
	return ticks;
}

u16 get_seconds(){
	return seconds;
}

void ticks_init(void){
	
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);
	TIM6->ARR = 1000;
	TIM6->PSC = 72 - 1;
	TIM6->EGR = 1;
	TIM6->SR = 0;
	TIM6->DIER = 1;
	TIM6->CR1 = 1;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;   
	NVIC_Init(&NVIC_InitStructure);
}

void TIM6_IRQHandler(void){
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	ticks ++;
	if( ticks >= 1000 ){
		ticks = 0;
		seconds ++;
	}	
}
