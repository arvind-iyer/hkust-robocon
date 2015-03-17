#include "ticks.h"

u16 ticks = 0;
u16 seconds =0;
void ticks_init(void){
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
	TIM1->ARR = 999;
	TIM1->PSC = SystemCoreClock / 1000000 - 1;
	TIM1->EGR = 1;
	TIM1->SR = 0;
	TIM1->DIER = 1;
	TIM1->CR1 = 1;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;   //??¤¤?
	NVIC_Init(&NVIC_InitStructure);
}

void TIM1_UP_IRQHandler (void){
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	
	ticks ++;
	if( ticks > 1000 ){
		ticks = 0;	
		seconds ++;
	}
  
  buzzer_check();
}

u16 get_ticks(void){
	return ticks;
}

u16 get_seconds(void){
	return seconds;
}
	
