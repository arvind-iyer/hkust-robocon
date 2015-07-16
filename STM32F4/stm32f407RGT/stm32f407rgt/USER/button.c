#include "stdio.h"
#include "button.h"

static const GPIO* buttons[BUTTON_COUNT] = { 
	SMALL_BUTTON_GPIO,
	JOY_UP_GPIO,
	JOY_CENTER_GPIO,
	JOY_LEFT_GPIO,
	JOY_RIGHT_GPIO,
	JOY_DOWN_GPIO
};



void button_init(){
	int i;
	for ( i = 0; i  < BUTTON_COUNT; i++)
	{
		BUTTON_init(buttons[i]);
	}
}

void button_update()
{
	int i;
	for (i = 0; i < BUTTON_COUNT; i++)
	{
		if(button_read(buttons[i]))
		{
			button_pressed_count[i]++;
		}
		else
		{
			button_pressed_count[i] = 0;
			button_released_count[i] = 1;
		}
		printf("%d : %d ",i, button_pressed_count[i]);
	}
}

u8 button_read(const GPIO* gpio)
{
	return gpio_read_input(gpio);
}

