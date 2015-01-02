#include "led.h"

void led_init(void)
{
	GPIO_InitTypeDef LED_InitStructure;
  
  RCC_APB2PeriphClockCmd(LED_RCC, ENABLE); 

	LED_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			   
	LED_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	LED_InitStructure.GPIO_Pin = LED_D1_Pin | LED_D2_Pin | LED_D3_Pin; 			    				
	GPIO_Init(LED_GPIO, &LED_InitStructure);

	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_ON);
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

