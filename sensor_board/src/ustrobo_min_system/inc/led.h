#ifndef	__LED_H
#define	__LED_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include "stm32f10x_tim.h"



#define	LED_D1_GPIO				((GPIO*) &PB2)

typedef enum {
	LED_D1 = 1 << 1,	// 1 (001)
} LED;
	
typedef enum {
	LED_OFF = 0,
	LED_ON	= 1
} LED_STATE;

void led_init(void);
void led_control(LED led, LED_STATE state);

#endif /* __LED_H */
