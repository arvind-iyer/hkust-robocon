#include "button.h"

/**
  * @brief  Initialization of button (joystick)'s gpio.
  * @param  None
  * @retval None
  */
void button_init(void)
{
	// RCC init
	RCC_APB2PeriphClockCmd(BUTTON_DIR_GPIO_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(BUTTON_CLICK_GPIO_RCC, ENABLE);

	// GPIO init
	gpio_init(*BUTTON_UP_GPIO, GPIO_Speed_2MHz, GPIO_Mode_IPU);
	gpio_init(*BUTTON_DOWN_GPIO, GPIO_Speed_2MHz, GPIO_Mode_IPU);
	gpio_init(*BUTTON_LEFT_GPIO, GPIO_Speed_2MHz, GPIO_Mode_IPU);
	gpio_init(*BUTTON_RIGHT_GPIO, GPIO_Speed_2MHz, GPIO_Mode_IPU);
	gpio_init(*BUTTON_CENTER_GPIO, GPIO_Speed_2MHz, GPIO_Mode_IPU);

}
