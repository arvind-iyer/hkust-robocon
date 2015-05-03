#ifndef	__NEC_PROTOCOL_H
#define	__NEC_PROTOCOL_H

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "gpio.h"

void NEC_init(void);


typedef struct {
	const GPIO* gpio;
	TIM_TypeDef* TIMx;
	u16 TIM_Channelx;
	u16 TIM_CCx;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
}	NEC_PORT_TypeDef;
#endif	/* __NEC_PROTOCOL_H */
