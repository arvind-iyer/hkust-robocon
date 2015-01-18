#ifndef __TICKS_H
#define __TICKS_H

#include "stm32f10x_tim.h"
#include "buzzer.h"

#define TICKS_TIM							TIM2
#define TICKS_RCC							RCC_APB1Periph_TIM2
#define TICKS_IRQn						TIM2_IRQn
#define TICKS_IRQHandler			void TIM2_IRQHandler(void)

//#define	TICKS_IRQHandler				void SysTick_Handler(void)
extern volatile u16 ticks;
extern volatile u16 seconds;

u16 get_ticks(void);
u16 get_seconds(void);
u32 get_full_ticks(void);
void ticks_init(void);

#endif		/*  __TICKS_H */
