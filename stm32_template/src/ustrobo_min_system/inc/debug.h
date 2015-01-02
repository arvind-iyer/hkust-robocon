#ifndef __DEBUG_H
#define __DEBUG_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#define BUZZER_PORT	GPIOF			
#define BUZZER_PIN	GPIO_Pin_0	
#define BUZZER_RCC	RCC_APB2Periph_GPIOF	

#define LED_R	1	   	// PG6
#define LED_G	2		// PG7
#define LED_B	4		// PD7

#define LED_ON 1
#define LED_OFF 0

void buzzer_init(void);	 						//init
void buzzer_control(u8 _count, u16 _period);	//unit of period is 10ms

void led_init(void);							//init
void led_control(u8 led, u8 state);				//led: LED_R , LED_G , LED_B ; state: LED_ON , LED_OFF

#endif /* __DEBUG_H */
