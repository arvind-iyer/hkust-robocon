#include "ticks.h"

volatile u16 ticks = 0;
volatile u16 seconds = 0;

/**
  * @brief  Get the ticks passed from 0-999
  * @param  None
  * @retval ticks passed
  */
u16 get_ticks() {
	return ticks;
}

/**
  * @brief  Get the seconds passed from
  * @param  seconds
  * @retval ticks passed
  */
u16 get_seconds() {
	return seconds;
}

/**
  * @brief  Initialization of ticks timer
  * @param  None
  * @retval None
  */
void ticks_init(void) {
//	NVIC_InitTypeDef NVIC_InitStructure;
//	RCC_APB1PeriphClockCmd(TICKS_RCC , ENABLE);
//	TICKS_TIM->PSC = SystemCoreClock / 1000000 - 1;		// Prescaler
//	TICKS_TIM->ARR = 1000;
//	TICKS_TIM->EGR = 1;
//	TICKS_TIM->SR = 0;
//	TICKS_TIM->DIER = 1;
//	TICKS_TIM->CR1 = 1;
//	
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStructure.NVIC_IRQChannel = TICKS_IRQn;
//	NVIC_Init(&NVIC_InitStructure);
	
	SysTick_Config(SystemCoreClock/1000);
	ticks = seconds = 0;
}

/**
  * @brief  Timer for ticks
  * @param  None
  * @retval None
  */
TICKS_IRQHandler
{
//	TIM_ClearFlag(TICKS_TIM, TIM_FLAG_Update);
//	TIM_ClearITPendingBit(TICKS_TIM, TIM_IT_Update);

	if (ticks >= 999) {
		ticks = 0;
		seconds++;
	} else {
		ticks++;
	}

	buzzer_check();
	
}

