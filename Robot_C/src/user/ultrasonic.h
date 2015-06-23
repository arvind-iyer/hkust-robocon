#include "gpio.h"
#include <stdbool.h>

#define ULTRASONIC_POWER &PB9
#define FORCE_DECEL_GPIO &PC4
#define FORCE_STOP_GPIO  &PC5

void ultrasonic_init(void);
bool is_force_decel(void);
bool is_force_stop(void);
