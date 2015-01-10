#include "ticks.h"

u32 ticks = 0;		//1 tick = 0.977 ms

#define ENCODER0 (u8) 0
#define ENCODER1 (u8) 1
#define ENCODER2 (u8) 2

void ticks_init(void){
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
	TIM1->ARR = 976;
	TIM1->PSC = 71;
	TIM1->EGR = 1;
	TIM1->SR = 0;
	TIM1->DIER = 1;
	TIM1->CR1 = 1;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

u16 get_ticks() {
	return ticks;
}

void TIM1_UP_IRQHandler (void)
{
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);	
	ticks = ++ticks >= 1024 ? 0 : ticks;

	hw_encoder_Count[ENCODER0] += hw_encoder_cal_count(ENCODER0);  //update the encoder0 count
	hw_encoder_Count[ENCODER1] += hw_encoder_cal_count(ENCODER1);  //update the encoder1 count
	hw_encoder_Count[ENCODER2] += hw_encoder_cal_count(ENCODER2);  //update the encoder2 count

	if (ticks % 4 == 0  && gyro_state == 2) {
		gyro_update();			
		calculation(T_SHAPE_SYSTEM);
	}
	
	if (buzzer_on) {
		if (buzzer_count-- % buzzer_period == 0) {
			buzzer_update();
		}
	}
}
