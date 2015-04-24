#ifndef __SENSORS_H
#define __SENSORS_H


#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "robocon.h"

#define laser_sensor_1 &PA4
#define laser_sensor_2 &PA6
#define laser_sensor_3 &PA7

// const variables
static const u32 SENSOR_PULSE_WIDTH_MAX = 100;		// in milliseconds * 2
static const u32 SENSOR_PULSE_WIDTH_MIN = 5;				// in milliseconds * 2


void sensors_update(void);

#endif
