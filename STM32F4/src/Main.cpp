/*
 * Main.cpp
 *
 *  Created on: 2015-04-02
 *      Author: William
 */

#include <inttypes.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <Ticks.h>
#include <Leds.h>
#include <Usart.h>
#include <buzzer_song.h>
#include <can_protocol.h>
#include "stm32f4xx_can.h"
#include <can_motor.h>
#include <stm32f4xx_it.h>
#include <cmath>
#include "encoder.h"
#include <stdio.h>
#include "motor.h"
#include "gpio.h"

static int ticks_img = 655;

int main()
{
	Ticks ticks;

	motor drived_motor(encoder_A, encoder_B, encoder_TIM, PWM_GPIO, DIR_A, DIR_B, PWM_TIM);
	Leds::leds_init();
	can_init();
	can_rx_init();
	can_motor_init();

	while (true) {
		if (ticks_img != ticks.getTicks()) {
			ticks_img = ticks.getTicks();

			// Send encoder value
			if (ticks_img % 100 == 0) {
				drived_motor.send_feedback();
			}

			// LED signal
			// Encoder work or not
			if (drived_motor.get_encoder_working_state()) {
				ENCODER_ERR_LED.off();
			} else {
				ENCODER_ERR_LED.on();
			}
			// Open or close loop or locking
		  if (drived_motor.get_close_loop_state()) {
				if (drived_motor.get_target_vel() == 0 && drived_motor.get_encoder_working_state()) {
					LOCK_LED.on();
				} else {
					LOCK_LED.off();
				}
				CLOSE_LOOP_LED.on();
				OPEN_LOOP_LED.off();
			} else {
				OPEN_LOOP_LED.on();
				CLOSE_LOOP_LED.off();
				LOCK_LED.off();
			}
			// life signal
			if (ticks_img % 100 == 0) {
				LIFE_LED.led_toggle();
			}
			// turn on CAN LED every seconds.
			if (ticks_img == 0) {
				NO_CAN_LED.on();
			}
		}
	}
}
