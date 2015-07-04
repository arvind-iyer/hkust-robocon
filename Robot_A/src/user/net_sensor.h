#ifndef	__NET_SENSOR_H
#define	__NET_SENSOR_H

#include "stm32f10x.h"
#include "gpio.h"

// 670 mm, 135cm, 190cm

#define	NET_SENSOR_Q1		((GPIO*) &PC2)
#define	NET_SENSOR_Q2		((GPIO*) &PC3) 

typedef enum {
	NET_SENSOR_FAR,
	NET_SENSOR_POINT1,
	NET_SENSOR_POINT2,
	NET_SENSOR_POINT3
} NET_SENSOR_MODE;


void net_sensor_init(void);
NET_SENSOR_MODE get_net_sensor(void);

#endif	/* __NET_SENSOR_H */
