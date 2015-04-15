/*
 * Leds.cpp
 *
 *  Created on: 2015-04-01
 *      Author: William
 */

#include <Leds.h>

Leds Signal10(LED_10), Signal11(LED_11), Signal12(LED_12);


Leds* led_array_ptr[] = {0, 0, 0, 0, 0, 0, 0, new Leds(LED_7), new Leds(LED_8)};

void Leds::led_control(LED_STATUS status) const
{
	gpio_m->gpio_write(status ? Bit_SET : Bit_RESET);
}

Leds::Leds(GPIO* const gpio, LED_STATUS init_state) : gpio_m(gpio)
{
	gpio_m->gpio_init(GPIO_Speed_100MHz, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	led_control(init_state);
}


void Leds::led_toggle() const
{
	led_control(gpio_m->gpio_read_output() ? LED_OFF : LED_ON);
}

void Leds::led_reinit(LED_STATUS init_state) const
{
	gpio_m->gpio_init(GPIO_Speed_100MHz, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	led_control(init_state);
}

LED::LED(Leds** led_array, unsigned int size) : led_no(size)
{
	led_array_m = new Leds*[size];
	for (int i = 0; i < size; ++i) {
		led_array_m[i] = led_array[i];
	}
}

void LED::multi_control(int bit) const
{

}


Leds* LED::operator[](unsigned int signal_n) const
{
	return led_array_m[signal_n];
}
