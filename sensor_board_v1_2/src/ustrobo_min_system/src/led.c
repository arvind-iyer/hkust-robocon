#include "led.h"

void led_init(void)
{
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
	gpio_init(LED_SIG1_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
	gpio_init(LED_SIG2_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
	gpio_init(LED_SIG3_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
	gpio_init(LED_SIG4_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
}


void led_control(LED led, LED_STATE state)
{
	
	if (led & LED_SIG1) {gpio_write(LED_SIG1_GPIO, (BitAction)state);}
	if (led & LED_SIG2) {gpio_write(LED_SIG2_GPIO, (BitAction)state);}
	if (led & LED_SIG3) {gpio_write(LED_SIG3_GPIO, (BitAction)state);}
	if (led & LED_SIG4) {gpio_write(LED_SIG4_GPIO, (BitAction)state);}
}

