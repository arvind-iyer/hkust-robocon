#ifndef PIN_TEST
#define PIN_TEST

#include <stdbool.h>
#include "stm32f10x_gpio.h"
#include "stm32f10x.h"
#include "led.h"
#include "lcd_red.h"
#include "ticks.h"
#include "buzzer.h"
#include "interface.h"
#include "battery.h"

typedef struct {
  u16 GPIOA_input, GPIOB_input, GPIOC_input, GPIOD_input, GPIOE_input;
} GPIO_Input;

void GPIO_init(void);
void pin_test(void);
GPIO_Input set_init(u16 A, u16 B, u16 C, u16 D, u16 E);

#endif  // PIN_TEST
