#ifndef __BATTERY_H
#define __BATTERY_H

#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "delay.h"

void battery_adc_init(void);
u8 sample_battery(void);

#endif /* __BATTERY_H */
