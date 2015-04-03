#include "led.h"

void led_init(void)
{
	gpio_init(LED_D1_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
	gpio_init(LED_D2_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
}


void led_control(LED led, LED_STATE state)
{
	
	if (led & LED_D1) {gpio_write(LED_D1_GPIO, (BitAction)state);}
	if (led & LED_D2) {gpio_write(LED_D2_GPIO, (BitAction)state);}

}

