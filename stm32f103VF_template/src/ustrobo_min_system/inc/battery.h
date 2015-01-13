#ifndef __BATTERY_H
#define __BATTERY_H

#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "delay.h"

#define BATTERY_VOLTAGE_1 1220 	//Voltage of first source
#define ADC_OUTPUT_1 			3682	//Value of "adc_value" for above voltage
#define BATTERY_VOLTAGE_2 1240 	//Voltage of second source
#define ADC_OUTPUT_2 			3760	//Value of "adc_value" for above voltage

//Following values taken from STM32 data sheet. Refer "Temperature Sensor characteristics"
#define AVG_SLOPE_TYP 		4300	//Typical Average Slope in microvolts
#define V25_TYP 					1430	//Typical V25 value in millivolts		

void adc_init(void);
u16 get_voltage(void);
u16 get_temp(void);


#endif /* __BATTERY_H */
