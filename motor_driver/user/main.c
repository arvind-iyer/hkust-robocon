/**
  ******************************************************************************
  * @file    main.c
  * @author  William LEE
  * @date    20-January-2015
  * @brief   This file run the main part of motor driver.
  ******************************************************************************
  * @attention
  *
  * C99 mode is enabled, please ensure your keil support it or error will caused.
	* Change the motor_id in can_motor.h for different motor.
	* Uncomment the DEBUG_MODE definition to enable debugging mode.
	* Systicks is used in stm32f10x_it.c, for high frequency control (5000Hz)
  ******************************************************************************
  */
#include "main.h"
//#define DEBUG_MODE

static u16 ticks_img = 65535;	//trivial value

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

int main(void)
{
	/** initialization **/
	system_init(5000);
	ticks_init();
	led_init();
	encoder_init();
	motor_init();
	can_init();
	can_rx_init();
	can_motor_init();
	uart_init(COM1, 115200);
	uart_printf_enable(COM1);
	/** end of init **/

	while (true) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			/** 1000Hz, can be higher by using systicks instead **/
			encoder_update();
			/** Depends on acceleration value to accelerate **/
			if (ticks_img % (1000 / get_current_accel()) == 0) {
				velocity_update();
			}
			/** end of motor control **/
			if (ticks_img % 100 == 0) {
				send_encoder(get_encoder());
			}
			
#ifdef DEBUG_MODE	// In debug mode, for hardware to debug by themselves
			debug();
#else							// Normal execute mode, led show life signal.
			/** flahsing led light to show mcu still working **/
			if (!FULL_SPEED_LIMIT) {
				life_signal();
			} else {
				led_control(LED_BOTH, LED_ON);
			}
#endif
		}
	}
}

