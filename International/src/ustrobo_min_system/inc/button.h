#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include "stm32f10x_tim.h"
#include "tft.h"
#include "xbc_mb.h"

#ifdef MAINBOARD_V4

/*****/
#define BUTTON_DOWN_LISTENER()    (button_pressed(BUTTON_JS_DOWN) == 1 || button_hold(BUTTON_JS_DOWN, 10, 1) || button_pressed(BUTTON_XBC_S) == 1 || button_hold(BUTTON_XBC_S, 10, 3))
#define BUTTON_UP_LISTENER()      (button_pressed(BUTTON_JS_UP) == 1 || button_hold(BUTTON_JS_UP, 10, 1) || button_pressed(BUTTON_XBC_N) == 1 || button_hold(BUTTON_XBC_N, 10, 3))
#define BUTTON_LEFT_LISTENER()    (button_pressed(BUTTON_JS_LEFT) == 1 || button_hold(BUTTON_JS_LEFT, 10, 1) || button_pressed(BUTTON_XBC_W) == 1 || button_hold(BUTTON_XBC_W, 10, 1))
#define BUTTON_RIGHT_LISTENER()   (button_pressed(BUTTON_JS_RIGHT) == 1 || button_hold(BUTTON_JS_RIGHT, 10, 1) || button_pressed(BUTTON_XBC_E) == 1 || button_hold(BUTTON_XBC_E, 10, 1))
#define BUTTON_ENTER_LISTENER()   (button_pressed(BUTTON_JS_CENTER) == 1 || button_pressed(BUTTON_XBC_START) == 1)
#define BUTTON_RETURN_LISTENER()   (button_pressed(BUTTON_1) == 5 || button_pressed(BUTTON_2) == 5 || button_pressed(BUTTON_XBC_BACK) == 5)

/****/

#define BUTTON_JS_UP_GPIO				((GPIO*) &PD0)
#define BUTTON_JS_LEFT_GPIO			((GPIO*) &PD1)
#define BUTTON_JS_DOWN_GPIO			((GPIO*) &PB8)
#define BUTTON_JS_RIGHT_GPIO		((GPIO*) &PD3)
#define BUTTON_JS_CENTER_GPIO		((GPIO*) &PD4)

#define	BUTTON_1_GPIO							((GPIO*) &PB6)
#define	BUTTON_2_GPIO							((GPIO*) &PB7)

#define BUTTON_RELEASED_LIMIT			20    /*!< Reset the button release time after the limit */

#define	BUTTON_COUNT							7    /*!< Number of buttons (exclude XBOX) */
#define XBC_BUTTON_START_ID       7
#define XBC_BUTTON_COUNTS         20

#else 

/*****/
#define BUTTON_DOWN_LISTENER()    (button_pressed(BUTTON_JS2_DOWN) == 1 || button_hold(BUTTON_JS2_DOWN, 10, 3) || button_pressed(BUTTON_XBC_S) == 1 || button_hold(BUTTON_XBC_S, 10, 3))
#define BUTTON_UP_LISTENER()      (button_pressed(BUTTON_JS2_UP) == 1 || button_hold(BUTTON_JS2_UP, 10, 3) || button_pressed(BUTTON_XBC_N) == 1 || button_hold(BUTTON_XBC_N, 10, 3))
#define BUTTON_LEFT_LISTENER()    (button_pressed(BUTTON_JS2_LEFT) == 1 || button_hold(BUTTON_JS2_LEFT, 10, 1) || button_pressed(BUTTON_XBC_W) == 1 || button_hold(BUTTON_XBC_W, 10, 1))
#define BUTTON_RIGHT_LISTENER()   (button_pressed(BUTTON_JS2_RIGHT) == 1 || button_hold(BUTTON_JS2_RIGHT, 10, 1) || button_pressed(BUTTON_XBC_E) == 1 || button_hold(BUTTON_XBC_E, 10, 1))
#define BUTTON_ENTER_LISTENER()   (button_pressed(BUTTON_JS2_CENTER) == 1 || button_pressed(BUTTON_XBC_START) == 1)
#define BUTTON_RETURN_LISTENER()   (button_pressed(BUTTON_1) == 5 || button_pressed(BUTTON_2) == 5 || button_pressed(BUTTON_XBC_BACK) == 5)

/****/


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

#define BUTTON_RELEASED_LIMIT			20    /*!< Reset the button release time after the limit */

#define	BUTTON_COUNT							12    /*!< Number of buttons (exclude XBOX) */
#define XBC_BUTTON_START_ID       12
#define XBC_BUTTON_COUNTS         20

#endif



typedef enum {
#ifdef MAINBOARD_V4
	BUTTON_JS_UP 				= 0,
	BUTTON_JS_LEFT			= 1,
	BUTTON_JS_DOWN			= 2,
	BUTTON_JS_RIGHT			= 3,
	BUTTON_JS_CENTER 		= 4, 
#else
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
#endif
	BUTTON_1,
	BUTTON_2,
 
  BUTTON_XBC_N		    = XBC_BUTTON_START_ID,
  BUTTON_XBC_S,
  BUTTON_XBC_W,
  BUTTON_XBC_E,
  
  BUTTON_XBC_START,
  BUTTON_XBC_BACK,
  BUTTON_XBC_L_JOY,
  BUTTON_XBC_R_JOY,
  
  BUTTON_XBC_LB,
  BUTTON_XBC_RB,
  BUTTON_XBC_XBOX,
  BUTTON_XBC_NULL,
  
  BUTTON_XBC_A,
  BUTTON_XBC_B,
  BUTTON_XBC_X,
  BUTTON_XBC_Y,
  
  BUTTON_XBC_NE,
  BUTTON_XBC_SE,
  BUTTON_XBC_SW,
  BUTTON_XBC_NW
  
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

