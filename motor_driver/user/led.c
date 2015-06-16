/**
  ******************************************************************************
  * @file    led.c
  * @author  William LEE
  * @version V3.5.0
  * @date    09-June-2015
  * @brief   This file provides the led control.
  ******************************************************************************
  * @attention
  *
	*
  ******************************************************************************
  */
#include "led.h"

/**
	* @brief 		led initialization
  * @param	  None
  * @retval 	None
	*/
void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	LED_RCC_init();
	GPIO_InitStructure.GPIO_Pin = LED_1_Pin | LED_2_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_GPIOx, &GPIO_InitStructure);

}

/**
	* @brief 		led controling
  * @param	  led: The value can be LED_1, LED_2, or LED_BOTH
	*	@param		state: The value can be either LED_ON or LED_OFF
  * @retval 	None
	*/
void led_control(LED led, LED_STATE state)
{
	u16 led_gpio_pin = 0;
	
	if (led & LED_1) {led_gpio_pin |= LED_1_Pin;}
	if (led & LED_2) {led_gpio_pin |= LED_2_Pin;}

	GPIO_WriteBit(LED_GPIOx, (u16) led_gpio_pin, (BitAction) state);
}

/**
	* @brief 		Ensure the mcu is operating by flashing led light
  * @param	  None
  * @retval 	None
	*/
void life_signal(void)
{
	if (get_seconds() % 2) {
		led_control(LED_1, LED_ON);
		led_control(LED_2, LED_OFF);
	} else {
		led_control(LED_1, LED_OFF);
		led_control(LED_2, LED_ON);
	}
}

/**
  * @brief  Warning signal, encoder is not working
  * @param  None
  * @retval None
  */
void encoder_malfunction_warning_signal(void)
{
  if (get_ticks() < 250 || get_ticks() > 750) {
    led_control(LED_BOTH, LED_ON);
  } else {
    led_control(LED_BOTH, LED_OFF);
  }
}

/**
  * @brief  Warning signal, motor is overspeed
  * @param  None
  * @retval None
  */
void motor_overspeed_signal(void)
{
	led_control(LED_BOTH, LED_ON);
}

