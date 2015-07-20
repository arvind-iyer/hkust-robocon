/*
 * Leds.h
 *
 *  Created on: 2015-04-01
 *      Author: William
 */

#ifndef LEDS_H_
#define LEDS_H_

#include <gpio.h>

GPIO* const OPEN_LOOP_GPIO = &PB5;
GPIO* const LIFE_GPIO = &PB6;
GPIO* const CLOSE_LOOP_GPIO = &PB7;
GPIO* const LOCK_GPIO = &PB9;
GPIO* const FIVE_VOLT_GPIO = &PB8;
GPIO* const ENCODER_ERR_GPIO = &PA2;
GPIO* const SPEED_10_GPIO = &PA6;
GPIO* const SPEED_30_GPIO = &PA5;
GPIO* const SPEED_50_GPIO = &PA3;
GPIO* const SPEED_70_GPIO = &PC2;
GPIO* const SPEED_90_GPIO = &PC0;
GPIO* const OVER_SPEED_GPIO = &PA1;

enum LED_STATUS {
	LED_ON = true,
	LED_OFF = false
};

class Leds {

	public:

		Leds(GPIO* const, LED_STATUS init_state = LED_OFF);
		void led_control(LED_STATUS status) const;
		void led_toggle() const;
		void on() const;
		void off() const;
		static void leds_init();

	private:
		void led_init(LED_STATUS init_state = LED_OFF) const;
		GPIO* const gpio_m;

};

extern const Leds OPEN_LOOP_LED, CLOSE_LOOP_LED,
			LOCK_LED, LIFE_LED,
			ENCODER_ERR_LED, SPEED_10_LED,
			SPEED_30_LED, SPEED_50_LED,
			SPEED_70_LED, SPEED_90_LED,
			OVER_SPEED_LED;


extern const Leds FIVE_VOLT_LED;
void speed_indicator_on(short id);
#endif /* LEDS_H_ */
