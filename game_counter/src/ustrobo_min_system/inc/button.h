#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include "stm32f10x_tim.h"
#include "tft.h"

#define BUTTON_1_GPIO				((GPIO*) &PA0)
#define BUTTON_2_GPIO				((GPIO*) &PA1)
#define BUTTON_4_GPIO				((GPIO*) &PA3)

#define	BUTTON_COUNT							3    /*!< Number of buttons */
#define BUTTON_RELEASED_LIMIT			20    /*!< Reset the button release time after the limit */

typedef enum {
	BUTTON_1,
  BUTTON_2,
  BUTTON_4
} BUTTON;

typedef enum {
	BUTTON_PRESSED			= 1,
	BUTTON_UNPRESSED 		= 0
} BUTTON_STATE;

void button_init(void);
void button_update(void);
u16 button_pressed(BUTTON b);
u8 button_hold(BUTTON b, u16 threshold, u8 mod);
u16 button_released(BUTTON b);


#endif /* __BUTTON_H */

