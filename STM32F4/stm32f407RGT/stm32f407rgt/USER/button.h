#include "gpio.h"

#define SMALL_BUTTON_GPIO   &PE3
#define JOY_UP_GPIO   		&PE5
#define JOY_CENTER_GPIO     &PC14
#define JOY_LEFT_GPIO   	&PE4
#define JOY_RIGHT_GPIO   	&PC13
#define JOY_DOWN_GPIO   	&PC15

#define BUTTON_COUNT 6

#define BUTTON_PRESS_TIME		20
#define BUTTON_RELEASE_TIME 20

typedef enum {
	SMALL_BUTTON = 0,
	JOY_UP = 1,
	JOY_CENTER = 2,
	JOY_LEFT = 3,
	JOY_RIGHT = 4,
	JOY_DOWN = 5
}BUTTON;

static const GPIO* buttons[BUTTON_COUNT] = { 
	SMALL_BUTTON_GPIO,
	JOY_UP_GPIO,
	JOY_CENTER_GPIO,
	JOY_LEFT_GPIO,
	JOY_RIGHT_GPIO,
	JOY_DOWN_GPIO
};


void button_init(void);
void button_update(void);
u8 button_pressed(BUTTON b);
u8 button_released(BUTTON b);
