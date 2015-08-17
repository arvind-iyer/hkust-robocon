#include "gpio.h"








void LED_init(const GPIO *gpio);
void LED_control(const GPIO *gpio, u8 state); //1:ON or 0:OFF
void LED_blink(const GPIO *gpio); //blink according to how frequent you call it
