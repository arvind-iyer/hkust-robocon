#ifndef	__INTERFACE_H
#define	__INTERFACE_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "tft.h"

#define	BATTERY_USB_LEVEL						600		// <= 6.00V will be treated as USB Power


void draw_battery_icon(u16 batt);


#endif /* __INTERFACE_H */
