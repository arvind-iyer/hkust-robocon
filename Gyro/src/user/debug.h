#ifndef __DEBUG_H
#define __DEBUG_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#define BUZZER_PORT GPIOA
#define BUZZER_PIN	GPIO_Pin_13
#define BUZZER_RCC	RCC_APB2Periph_GPIOA

extern u8 buzzer_on;
extern u16 buzzer_period;
extern u32 buzzer_count;

void buzzer_init(void);	 						//init
void buzzer_control(u8 _count, u16 _period);	//unit of period is 10ms
void buzzer_update(void);

#endif /* __DEBUG_H */
