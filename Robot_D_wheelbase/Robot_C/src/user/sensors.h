#ifndef __SENSORS_H
#define __SENSORS_H


#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "robocon.h"


// sensor arrays. init_IPD, 0 or floating.
#define laser_sensor_1 &PA5
#define laser_sensor_2 &PA6
#define laser_sensor_3 &PA7
#define laser_sensor_4 &PA7
#define laser_sensor_5 &PA7
#define laser_sensor_6 &PA7
#define laser_sensor_7 &PA7

// const variables
static const u32 ULTRA_DETECT_MIN_RANGE = 600;
static const u32 ULTRA_DETECT_MAX_RANGE = 1500;
static const u32 SENSOR_PULSE_WIDTH_MAX = 200;		// in milliseconds
static const u32 SENSOR_PULSE_WIDTH_MIN = 10;				// in milliseconds


void sensors_update(void);

#endif
