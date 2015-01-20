#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include "stm32f10x_tim.h"

#define BUTTON_J1_UP_GPIO				((GPIO*) &PD0)
#define BUTTON_J1_LEFT_GPIO			((GPIO*) &PD1)
#define BUTTON_J1_DOWN_GPIO			((GPIO*) &PB8)
#define BUTTON_J1_RIGHT_GPIO		((GPIO*) &PD3)
#define BUTTON_J1_CENTER_GPIO		((GPIO*) &PD5)

#define BUTTON_J2_UP_GPIO				((GPIO*) &PA4)
#define BUTTON_J2_LEFT_GPIO			((GPIO*) &PA5)
#define BUTTON_J2_DOWN_GPIO			((GPIO*) &PA6)
#define BUTTON_J2_RIGHT_GPIO		((GPIO*) &PA7)
#define BUTTON_J2_CENTER_GPIO		((GPIO*) &PC4)

#define	BUTTON_COUNT						10


typedef enum {
	BUTTON_J1_UP 			= 1 << 0,
	BUTTON_J1_LEFT		= 1 << 1,
	BUTTON_J1_DOWN		= 1 << 2,
	BUTTON_J1_RIGHT		= 1 << 3,
	BUTTON_J1_CENTER 	= 1 << 4, 
	BUTTON_J2_UP 			= 1 << 5,
	BUTTON_J2_LEFT		= 1 << 6,
	BUTTON_J2_DOWN		= 1 << 7,
	BUTTON_J2_RIGHT		= 1 << 8,
	BUTTON_J2_CENTER	= 1 << 9

} BUTTON;

void button_init(void);

#endif /* __BUTTON_H */

