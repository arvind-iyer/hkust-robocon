#include "gpio.h"

void LED_init(const GPIO *gpio);
void LED_control(const GPIO *gpio, LED_STATE state);