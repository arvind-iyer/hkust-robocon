#ifndef _ticks_h_
#define _ticks_h_

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "buzzer.h"

extern u16 ticks;

void ticks_init(void);
u16 get_ticks(void);
u16 get_seconds(void);

#endif
