#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"
#include "stm32f10x_tim.h"

extern s32 hw_encoder_Count[3];

void hw_encoder_Init(void);
void encoder_mode(TIM_TypeDef* ENCODER_TIMER);
s16 hw_encoder_cal_count(u8 motor);

#endif		/* __ENCODER_H */

