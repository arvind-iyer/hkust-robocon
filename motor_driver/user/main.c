/**
  ******************************************************************************
  * @file    main.c
  * @author  William LEE
  * @date    09-June-2015
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
	/** end of init **/

	while (true) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			/** 1000Hz, can be higher by using systicks instead **/
			encoder_update();
			/** Accelerate for every milisecond if motor not reach our velocity setting**/
			velocity_update();
			/** end of motor control **/
			if (ticks_img % 5 == 0) {
				send_encoder(get_encoder());
			}
			
#ifdef DEBUG_MODE	// In debug mode, for hardware to debug by themselves
			debug();
#else							// Normal execute mode, led show life signal.
			/** flahsing led light to show mcu still working **/
			if (!is_encoder_working()) {
        encoder_malfunction_warning_signal();
      } else if (is_overspeed()) {
				motor_overspeed_signal();
			} else {
				life_signal();
			}
#endif
		}
	}
}

