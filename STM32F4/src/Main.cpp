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
			drived_motor.encoder::refresh();
			drived_motor.refresh();
			if (ticks_img % 100 == 0) {
				drived_motor.send_feedback();
			}
#warning "Wrong LED"
#define TESTING
			// LED signal
			// Encoder
#ifdef TESTING
			static int sec = 0;
			if (Ticks::getInstance()->getSeconds() != sec) {
				sec = Ticks::getInstance()->getSeconds();
				ENCODER_ERR_LED.led_toggle();
			}


			switch (static_cast<int>(std::abs(drived_motor.get_current_vel()))) {
				case 0 ... 35:
					speed_indicator_on(0);
					break;
				case 36 ... 70:
					speed_indicator_on(1);
					break;
				case 81 ... 105:
					speed_indicator_on(2);
					break;
				case 106 ... 140:
					speed_indicator_on(3);
					break;
				case 141 ... 175:
					speed_indicator_on(4);
					break;
				case 176 ... 200:
					speed_indicator_on(5);
					break;
				default:
					speed_indicator_on(6);
			}

#else
			if (!drived_motor.is_encoder_working()) {
				ENCODER_ERR_LED.on();
			} else {
				ENCODER_ERR_LED.off();
			}
#endif
			// Open or close loop or locking
			if (drived_motor.is_open_loop()) {
				OPEN_LOOP_LED.on();
				CLOSE_LOOP_LED.off();
				LOCK_LED.off();
			} else {
				if (drived_motor.get_current_vel() == 0 && drived_motor.is_encoder_working()) {
					LOCK_LED.on();
				} else {
					LOCK_LED.off();
				}
				CLOSE_LOOP_LED.on();
				OPEN_LOOP_LED.off();
			}
			// life signal
			if (ticks_img % 500 == 0) {
				LIFE_LED.led_toggle();
			}

			if (ticks_img == 0) {
				FIVE_VOLT_LED.on();
			}
		}
	}
}
