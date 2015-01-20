#include "button.h"

static const GPIO* buttons[BUTTON_COUNT] = {
	BUTTON_J1_UP_GPIO,
	BUTTON_J1_LEFT_GPIO,
	BUTTON_J1_DOWN_GPIO,
	BUTTON_J1_RIGHT_GPIO,
	BUTTON_J1_CENTER_GPIO,
	BUTTON_J2_UP_GPIO,
	BUTTON_J2_LEFT_GPIO,
	BUTTON_J2_DOWN_GPIO,
	BUTTON_J2_RIGHT_GPIO,
	BUTTON_J2_CENTER_GPIO
};


/**
  * @brief  Initialization of button (joystick)'s gpio.
  * @param  None
  * @retval None
  */
void button_init(void)
{
	// GPIO init
	gpio_rcc_init(buttons[0]);
	for (u8 i = 0; i < sizeof(buttons) / sizeof(GPIO*); ++i) {
		gpio_init(buttons[i], GPIO_Speed_2MHz, GPIO_Mode_IPU, 0);
	}
	
}

void button_update(void)
{
	
}
