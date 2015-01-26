/**
  ******************************************************************************
  * @file    debug.c
  * @author  William LEE
  * @version V3.5.0
  * @date    24-January-2015
  * @brief   This file provides debug tools for testing the motor controller
  ******************************************************************************
  * @attention
  *
  * Call the debug function inside while in main.c when used.
	*
  ******************************************************************************
  */
#include "debug.h"

/**
	* @brief	Auto generate velocity by timming for motor controller testing
	*	@param	None.
	*	@retval	None.
	*/
void debug(void)
{
	if (get_seconds() < 5) {
		set_pwm(0);
		led_control(LED_BOTH, (get_ticks() < 500 ? LED_OFF : LED_ON));
	} else if (get_seconds() < 10) {
		led_control(LED_1, LED_ON);
		led_control(LED_2, LED_OFF);
		set_velocity(50);
	} else if (get_seconds() < 15) {
		led_control(LED_2, LED_ON);
		set_velocity(0);	// motor_lock
	} else if (get_seconds() < 20) {
		led_control(LED_1, LED_OFF);
		set_velocity(-50);
	} else {
		set_pwm(0);
		led_control(LED_BOTH, LED_OFF);
	}
}
