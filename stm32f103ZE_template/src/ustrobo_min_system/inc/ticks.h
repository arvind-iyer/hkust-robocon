#ifndef __TICKS_H
#define __TICKS_H

#include "stm32f10x_tim.h"

extern volatile u16 ticks;
extern volatile u16 seconds;

u16 get_ticks(void);
u16 get_seconds(void);
void ticks_init(void);

#endif		/*  __TICKS_H */
