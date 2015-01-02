#ifndef	__LED_H
#define	__LED_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#define	LED_GPIO		GPIOC
#define	LED_RCC			RCC_APB2Periph_GPIOC
#define	LED_D1_Pin	GPIO_Pin_1
#define	LED_D2_Pin	GPIO_Pin_2
#define	LED_D3_Pin	GPIO_Pin_3

typedef enum {
	LED_D1 = 1 << 1,	// 1 (001)
	LED_D2 = 1 << 2,	// 2 (010)
	LED_D3 = 1 << 3		// 4 (100)
} LED;
	
typedef enum {
	LED_OFF = 0,
	LED_ON	= 1
} LED_STATE;

void led_init(void);
void led_control(LED led, LED_STATE state);

#endif /* __LED_H */
