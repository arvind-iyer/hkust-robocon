#include "ultrasonic.h"

void ultrasonic_init(void)
{
	gpio_write(ULTRASONIC_POWER, Bit_RESET);
	gpio_init(FORCE_DECEL_GPIO, GPIO_Speed_50MHz, GPIO_Mode_IPD, 1);
	gpio_init(FORCE_STOP_GPIO, GPIO_Speed_50MHz, GPIO_Mode_IPD, 1);	
}

bool is_force_decel(void)
{
	return gpio_read_input(FORCE_DECEL_GPIO);
}

bool is_force_stop(void)
{
	return gpio_read_input(FORCE_STOP_GPIO);
}
