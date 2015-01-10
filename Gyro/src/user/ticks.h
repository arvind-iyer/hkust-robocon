#ifndef __TICKS_H
#define __TICKS_H

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "gyro.h"
#include "encoder.h"
#include "algorithm.h"
#include "debug.h"

void ticks_init(void);
u16 get_ticks(void);

#endif
