#include "net_sensor.h"
#include <stdbool.h>

static NET_SENSOR_MODE current_mode = NET_SENSOR_FAR;

void net_sensor_init(void)
{
	gpio_init(NET_SENSOR_Q1, GPIO_Speed_2MHz, GPIO_Mode_IPD, 1);
	gpio_init(NET_SENSOR_Q2, GPIO_Speed_2MHz, GPIO_Mode_IPD, 1);
	
	current_mode = NET_SENSOR_FAR;
}

NET_SENSOR_MODE get_net_sensor(void)
{
	bool q1 = gpio_read_input(NET_SENSOR_Q1), q2 = gpio_read_input(NET_SENSOR_Q2);
	
	if (q1 && !q2) {
		current_mode = NET_SENSOR_POINT1;
	} else if (q1 && q2) {
		current_mode = NET_SENSOR_POINT2;
	} else if (!q1 && q2) {
		current_mode = NET_SENSOR_POINT3;
	} else {
		current_mode = NET_SENSOR_FAR;
	}
	return current_mode;
}

