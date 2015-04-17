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

#define BUTTON_E0_GPIO                 ((GPIO*) &PE0)
#define BUTTON_E1_GPIO                 ((GPIO*) &PE1)
#define BUTTON_E2_GPIO                 ((GPIO*) &PE2)
#define BUTTON_E3_GPIO                 ((GPIO*) &PE3)
#define BUTTON_E4_GPIO                 ((GPIO*) &PE4)
#define BUTTON_E5_GPIO                 ((GPIO*) &PE5)
#define BUTTON_E6_GPIO                 ((GPIO*) &PE6)
#define BUTTON_E7_GPIO                 ((GPIO*) &PE7)
#define BUTTON_E8_GPIO                 ((GPIO*) &PE8)
#define BUTTON_E9_GPIO                 ((GPIO*) &PE9)
#define BUTTON_E10_GPIO                ((GPIO*) &PE10)
#define BUTTON_E11_GPIO                ((GPIO*) &PE11)
#define BUTTON_E12_GPIO                ((GPIO*) &PE12)
#define BUTTON_E13_GPIO                ((GPIO*) &PE13)
#define BUTTON_E14_GPIO                ((GPIO*) &PE14)
#define BUTTON_E15_GPIO                ((GPIO*) &PE15)
#define BUTTON_D8_GPIO                 ((GPIO*) &PD8)
#define BUTTON_D9_GPIO                 ((GPIO*) &PD9)
#define BUTTON_D10_GPIO                ((GPIO*) &PD10)
//#define BUTTON_D11_GPIO                ((GPIO*) &PD11)
#define	BUTTON_B9_GPIO								 ((GPIO*) &PB9)

#define	BUTTON_COUNT							32    /*!< Number of buttons */
#define BUTTON_RELEASED_LIMIT			20    /*!< Reset the button release time after the limit */

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
  
  BUTTON_E0,
  BUTTON_E1,
  BUTTON_E2,
  BUTTON_E3,
  BUTTON_E4,
  BUTTON_E5,
  BUTTON_E6,
  BUTTON_E7,
  BUTTON_E8,
  BUTTON_E9,
  BUTTON_E10,
  BUTTON_E11,
  BUTTON_E12,
  BUTTON_E13,
  BUTTON_E14,
  BUTTON_E15,
  
  BUTTON_D8,
  BUTTON_D9,
  BUTTON_D10,
	
	BUTTON_B9
  
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

