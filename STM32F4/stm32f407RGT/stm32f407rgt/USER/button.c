#include "stdio.h"
#include "button.h"

static u16 button_pressed_count[BUTTON_COUNT];
static u16 button_released_count[BUTTON_COUNT];


u8 button_read(const GPIO* gpio)
{
	return !gpio_read_input(gpio);
}

/**
  * @brief  : Initialises all buttons in list
  * @param  : None
  * @retval : None
  */

void button_init(){
	int i;
	for ( i = 0; i  < BUTTON_COUNT; i++)
	{
		BUTTON_init(buttons[i]);
	}
}

/**
  * @brief  : Iteratively updates the state of each button
  * @param  : None
  * @retval : None
  */
void button_update()
{
	for (int b = 0; b < BUTTON_COUNT; b++)
	{
		if(button_read(buttons[b]) == 1)
		{
			button_pressed_count[b] += button_pressed_count[b] == BUTTON_RELEASE_TIME ? 0 : 1;
			button_released_count[b] = 0;
		}
		else
		{
			if(button_pressed_count[b] > 0)
			{
				button_released_count[b] = 1;
			}
			if(button_released_count[b] && button_released_count[b] < BUTTON_RELEASE_TIME)
				button_released_count[b]++;
			else if(button_released_count[b] == BUTTON_RELEASE_TIME)
				button_released_count[b] = 0;
			
			button_pressed_count[b] = 0;
		}
		
		
	}
}


/**
  * @brief  : Returns button pressed state
  * @param  : Button enum
  * @retval : Button state
  */
u8 button_pressed(BUTTON b)
{
	return button_pressed_count[b];
}


/**
  * @brief  : Returns button released state
  * @param  : Button enum
  * @retval : Button state
  */
u8 button_released(BUTTON b)
{
	return button_released_count[b];
}
