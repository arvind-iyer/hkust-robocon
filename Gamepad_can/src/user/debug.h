#ifndef __DEBUG_H
#define __DEBUG_H

#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "delay.h"					   
#include "main.h"
													
#define BUZZER_PORT GPIOA			
#define BUZZER_PIN	GPIO_Pin_1	
#define BUZZER_RCC	RCC_APB2Periph_GPIOA	


void buzzer_init(void);	 						//init
void buzzer_control(u8 count, u8 period);		//unit of period is 10ms

#endif /* __DEBUG_H */
