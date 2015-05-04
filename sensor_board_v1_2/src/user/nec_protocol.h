#ifndef	__NEC_PROTOCOL_H
#define	__NEC_PROTOCOL_H

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "gpio.h"
#include "nec.h"


#define NEC_TIMER_PERIOD			65535
void NEC_init(void);


typedef struct {
	const GPIO* gpio;
	TIM_TypeDef* TIMx;
	u16 TIM_Channelx;
	u16 TIM_CCx;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	// Temp data storage
	u8 last_gpio_state;
	u16 deque[NEC_QUEUE_SIZE];
	u16 deque_head, deque_tail;
	
}	NEC_PORT_TypeDef;

void NEC_init(void);

#endif	/* __NEC_PROTOCOL_H */
