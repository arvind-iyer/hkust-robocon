#include "gpio.h"
#include <stdbool.h>
#include "adc.h"

#define ULTRASONIC_POWER &PB9
#define FORCE_DECEL_GPIO &PC4
#define FORCE_STOP_GPIO  &PC5

#define	ULTRASONIC_ADC_CHANNEL	15 

#define	ULTRASONIC_ADC_GPIO		(&PC5)

void ultrasonic_init(void);
u16 ultrasonic_get_val(void);

bool is_force_decel(void);
bool is_force_stop(void);
