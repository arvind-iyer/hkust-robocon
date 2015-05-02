#ifndef	__LED_H
#define	__LED_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include "stm32f10x_tim.h"



#define	LED_SIG1_GPIO				((GPIO*) &PB4)
#define	LED_SIG2_GPIO				((GPIO*) &PB5)
#define	LED_SIG3_GPIO				((GPIO*) &PB6)
#define	LED_SIG4_GPIO				((GPIO*) &PB7)

typedef enum {
	LED_SIG1 = 1 << 1,	// 1 (001)
	LED_SIG2 = 1 << 2,	// 1 (001)
	LED_SIG3 = 1 << 3,	// 1 (001)
	LED_SIG4 = 1 << 4,	// 1 (001)
} LED;
	
typedef enum {
	LED_OFF = 0,
	LED_ON	= 1
} LED_STATE;

void led_init(void);
void led_control(LED led, LED_STATE state);

#endif /* __LED_H */
