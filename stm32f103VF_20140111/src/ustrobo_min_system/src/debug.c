#include "debug.h"



/**
  * @brief  Initialaztion of LED
  * @param  None
  * @retval None
  */
void led_init(void)
{
	GPIO_InitTypeDef LED_InitStructure;
  
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOG, ENABLE); 

	LED_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;			   
  	LED_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	LED_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 			    				
  	GPIO_Init(GPIOG, &LED_InitStructure);

  	LED_InitStructure.GPIO_Pin = GPIO_Pin_7; 			    				
  	GPIO_Init(GPIOD, &LED_InitStructure);
	
	led_control(LED_R | LED_G | LED_B, LED_OFF);
}

/**
  * @brief  Turn on/off the LED
  * @param  led: which LEDs to be turned on/off (LED_R, LED_G, LED_B)
  * @param  state: turn on/off (LED_ON, LED_OFF)
  * @retval None
  */
void led_control(u8 led, u8 state)
{
	if (led & LED_R) {
		if (state)
			GPIO_ResetBits(GPIOG, GPIO_Pin_6);
		else  
			GPIO_SetBits(GPIOG, GPIO_Pin_6);
	}
	if (led & LED_G) {
		if (state)
			GPIO_ResetBits(GPIOG, GPIO_Pin_7);
		else  
			GPIO_SetBits(GPIOG, GPIO_Pin_7);
	}
	if (led & LED_B) {
		if (state)
			GPIO_ResetBits(GPIOD, GPIO_Pin_7);
		else  
			GPIO_SetBits(GPIOD, GPIO_Pin_7);
	}
}
