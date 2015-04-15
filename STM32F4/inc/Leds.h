/*
 * Leds.h
 *
 *  Created on: 2015-04-01
 *      Author: William
 */

#ifndef LEDS_H_
#define LEDS_H_

#include <gpio.h>

GPIO* const LED_[] = {&PA3, &PA4, &PC5, &PB0, &PB1, &PB10, &PB11, &PB12, &PB13, &PC10, &PC12, &PC11};

GPIO* const LED_1 = &PA3;
GPIO* const LED_2 = &PA4;
GPIO* const LED_3 = &PC5;
GPIO* const LED_4 = &PB0;
GPIO* const LED_5 = &PB1;
GPIO* const LED_6 = &PB10;
GPIO* const LED_7 = &PB11;
GPIO* const LED_8 = &PB12;
GPIO* const LED_9 = &PB13;
GPIO* const LED_10 = &PC10;
GPIO* const LED_11 = &PC12;
GPIO* const LED_12 = &PC11;

enum LED_STATUS {
	LED_ON = true,
	LED_OFF = false
};

class Leds {

	public:

		Leds(GPIO* const, LED_STATUS init_state = LED_OFF);
		void led_reinit(LED_STATUS init_state = LED_OFF) const;
		void led_control(LED_STATUS status) const;
		void led_toggle() const;

	private:
		GPIO* const gpio_m;

};

class LED {
public:
	LED(Leds** led_array, unsigned int size);
	void multi_control(int bit) const;
	Leds* operator[](unsigned int) const;
private:
	Leds** led_array_m;
	const int led_no;

};


extern Leds* led_array_ptr[];
extern Leds Signal10, Signal11, Signal12;

#endif /* LEDS_H_ */
