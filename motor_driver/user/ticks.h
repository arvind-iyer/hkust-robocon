#ifndef __TICKS_H
#define __TICKS_H

#include "stm32f10x_tim.h"
#include "system_stm32f10x.h"
#include "misc.h"

extern volatile u16 main_ticks;
extern volatile u16 seconds;

void ticks_init(void);
u16 get_ticks(void);
u16 get_seconds(void);

void TIM4_IRQHandler(void);
#endif		/*  __TICKS_H */
