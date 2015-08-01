/*
 * Leds.cpp
 *
 *  Created on: 2015-04-01
 *      Author: William
 */

#include <Leds.h>

//Leds FIVE_VOLT_LED(FIVE_VOLT_GPIO);

const Leds OPEN_LOOP_LED(OPEN_LOOP_GPIO), CLOSE_LOOP_LED(CLOSE_LOOP_GPIO),
		LOCK_LED(LOCK_GPIO), LIFE_LED(LIFE_GPIO), FIVE_VOLT_LED(FIVE_VOLT_GPIO),
		ENCODER_ERR_LED(ENCODER_ERR_GPIO), SPEED_10_LED(SPEED_10_GPIO),
		SPEED_30_LED(SPEED_30_GPIO), SPEED_50_LED(SPEED_50_GPIO),
		SPEED_70_LED(SPEED_70_GPIO), SPEED_90_LED(SPEED_90_GPIO),
		OVER_SPEED_LED(OVER_SPEED_GPIO);

void Leds::led_control(LED_STATUS status) const
{
	gpio_m->gpio_write(status ? Bit_SET : Bit_RESET);
}

Leds::Leds(GPIO* const gpio, LED_STATUS init_state) : gpio_m(gpio)
{
	gpio_m->gpio_init(GPIO_Speed_100MHz, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	led_control(init_state);
}

void Leds::on() const
{
	gpio_m->gpio_write(Bit_SET);
}

void Leds::off() const
{
	gpio_m->gpio_write(Bit_RESET);
}

void Leds::led_toggle() const
{
	led_control(gpio_m->gpio_read_output() ? LED_OFF : LED_ON);
}

void Leds::led_init(LED_STATUS init_state) const
{
	gpio_m->gpio_init(GPIO_Speed_100MHz, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	led_control(init_state);
}

void Leds::leds_init()
{
	OPEN_LOOP_LED.led_init();
	CLOSE_LOOP_LED.led_init(),
	LOCK_LED.led_init();
	LIFE_LED.led_init();
	ENCODER_ERR_LED.led_init();
	SPEED_10_LED.led_init();
	SPEED_30_LED.led_init();
	SPEED_50_LED.led_init();
	SPEED_70_LED.led_init();
	SPEED_90_LED.led_init();
	OVER_SPEED_LED.led_init();
	FIVE_VOLT_LED.led_init();
}

static void speed_indicator_off(short id) {
	switch (id) {
		case 0:
			break;
		case 1:
			SPEED_10_LED.off();
			break;
		case 2:
			SPEED_30_LED.off();
			break;
		case 3:
			SPEED_50_LED.off();
			break;
		case 4:
			SPEED_70_LED.off();
			break;
		case 5:
			SPEED_90_LED.off();
			break;
		case 6:
			OVER_SPEED_LED.off();
			break;
	}
}

void speed_indicator_on(short id)
{
	static short last_led = -1;
	if (last_led != id) {
		switch (id) {
			case 0:
				break;
			case 1:
				SPEED_10_LED.on();
				break;
			case 2:
				SPEED_30_LED.on();
				break;
			case 3:
				SPEED_50_LED.on();
				break;
			case 4:
				SPEED_70_LED.on();
				break;
			case 5:
				SPEED_90_LED.on();
				break;
			case 6:
				OVER_SPEED_LED.on();
				break;
		}
		speed_indicator_off(last_led);
		last_led = id;
	}
}
