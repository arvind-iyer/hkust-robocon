#ifndef	__BUZZER_H
#define	__BUZZER_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#define BUZZER_PORT					GPIOB
#define BUZZER_PIN					GPIO_Pin_0	
#define BUZZER_RCC					RCC_APB2Periph_GPIOB

#define BUZZER_TIM					TIM3
#define BUZZER_TIM_RCC			RCC_APB1Periph_TIM3
#define BUZZER_TIM_REMAP		GPIO_FullRemap_TIM3


void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_control(u8 count, u16 period);
void buzzer_check(void);

#endif	/* __BUZZER_H */
