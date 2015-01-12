#include "led.h"

void led_init(void)
{
	gpio_rcc_init(*LED_D1_GPIO);
	gpio_rcc_init(*LED_D2_GPIO);
	gpio_rcc_init(*LED_D3_GPIO);
	
	gpio_init(*LED_D1_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP);
	gpio_init(*LED_D2_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP);
	gpio_init(*LED_D3_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP);
}

// led_control(LED_D2 | LED_D3, LED_ON);
void led_control(LED led, LED_STATE state)
{
	u16 led_gpio_pin = 0;
	
	if (led & LED_D1) {led_gpio_pin |= LED_D1_Pin;}
	if (led & LED_D2) {led_gpio_pin |= LED_D2_Pin;}
	if (led & LED_D3) {led_gpio_pin |= LED_D3_Pin;}

	GPIO_WriteBit(LED_GPIO, (u16) led_gpio_pin, (BitAction) state);
}

