#ifndef __ADC_APP_H
#define __ADC_APP_H

#include "stm32f10x.h"
#include "adc.h"
#include "gpio.h"

/*** BATTERY VOLTAGE ***/
#define BATTERY_GPIO                ((GPIO*) &PC0)
#define SAMPLE_BATTERY_VOLTAGE_1 		1200 	// Voltage of first source
#define SAMPLE_BATTERY_ADC_1 				3620	// Value of "adc_value" for above voltage
#define SAMPLE_BATTERY_VOLTAGE_2		1100 	// Voltage of second source
#define SAMPLE_BATTERY_ADC_2	 			3310	// Value of "adc_value" for above voltage

#define	BATTERY_USB_LEVEL						600		// <= 6.00V will be treated as USB Power
#define	BATTERY_LOW_LEVEL						1160	// <= 11.50V will generate warning sound
#define	BATTERY_SUPER_LOW_LEVEL			1120	// <= 11.10V will stop the program (while loop)

#define BATTERY_ADC_CHANNEL         ADC_Channel_10

#define LASER_ADC_CHANNEL           ADC_Channel_14


/*** TEMPERATURE SENSOR ***/
#define SAMPLE_TEMP_VAL_1             200 // x10
#define SAMPLE_TEMP_ADC_1             1750
#define SAMPLE_TEMP_VAL_2             800 // x10
#define SAMPLE_TEMP_ACD_2             1450

typedef enum {
	BATTERY_OKAY,
	BATTERY_LOW,
	BATTERY_SUPER_LOW,
	BATTERY_USB
} BATTERY_CHECK_RESULT;

void battery_init(void);
s16 get_voltage(void);
BATTERY_CHECK_RESULT battery_check(void);
s16 get_temperature(void);
u32 get_sensor(void);

#endif  /** __ADC_APP_H **/
