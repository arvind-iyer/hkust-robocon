#ifndef __TICKS_H
#define __TICKS_H

#include "stm32f10x_tim.h"
#include "system_stm32f10x.h"
#include "misc.h"

#define TICKS_TIM							TIM4
#define TICKS_RCC							RCC_APB1Periph_TIM4
#define TICKS_IRQn						TIM4_IRQn
#define TICKS_IRQHandler			void TIM4_IRQHandler(void)

void ticks_init(void);
u16 get_ticks(void);
u16 get_seconds(void);
void TIM4_IRQHandler(void);

#endif		/*  __TICKS_H */
