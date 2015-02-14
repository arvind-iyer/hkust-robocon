#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include "stm32f10x_tim.h"
#include "tft.h"

#define BUTTON_JS1_UP_GPIO				((GPIO*) &PD0)
#define BUTTON_JS1_LEFT_GPIO			((GPIO*) &PD1)
#define BUTTON_JS1_DOWN_GPIO			((GPIO*) &PB8)
#define BUTTON_JS1_RIGHT_GPIO			((GPIO*) &PD3)
#define BUTTON_JS1_CENTER_GPIO		((GPIO*) &PD4)

#define BUTTON_JS2_UP_GPIO				((GPIO*) &PA4)
#define BUTTON_JS2_LEFT_GPIO			((GPIO*) &PA5)
#define BUTTON_JS2_DOWN_GPIO			((GPIO*) &PA6)
#define BUTTON_JS2_RIGHT_GPIO			((GPIO*) &PA7)
#define BUTTON_JS2_CENTER_GPIO		((GPIO*) &PC4)

#define	BUTTON_1_GPIO							((GPIO*) &PB6)
#define	BUTTON_2_GPIO							((GPIO*) &PB7)

#define	BUTTON_COUNT							14    /*!< Number of buttons */
#define BUTTON_RELEASED_LIMIT			20    /*!< Reset the button release time after the limit */

/** switch **/
#define ROTATE_SWITCH_GPIO        ((GPIO*) &PE3)
#define RACKET_SWITCH_GPIO        ((GPIO*) &PE2)

typedef enum {
	BUTTON_JS1_UP 			= 0,
	BUTTON_JS1_LEFT			= 1,
	BUTTON_JS1_DOWN			= 2,
	BUTTON_JS1_RIGHT		= 3,
	BUTTON_JS1_CENTER 	= 4, 
	BUTTON_JS2_UP 			= 5,
	BUTTON_JS2_LEFT			= 6,
	BUTTON_JS2_DOWN			= 7,
	BUTTON_JS2_RIGHT		= 8,
	BUTTON_JS2_CENTER		= 9,
	BUTTON_1						= 10,
	BUTTON_2						= 11,
  ROTATE_SWITCH       = 12,
  RACKET_SWITCH       = 13
} BUTTON;

typedef enum {
	BUTTON_PRESSED			= 0,
	BUTTON_UNPRESSED 		= 1
} BUTTON_STATE;

void button_init(void);
void button_update(void);
u16 button_pressed(BUTTON b);
u8 button_hold(BUTTON b, u16 threshold, u8 mod);
u16 button_released(BUTTON b);


#endif /* __BUTTON_H */

