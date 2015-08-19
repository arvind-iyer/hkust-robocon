#include "adc.h"
#include "stm32f4xx.h"
#include "ticks.h"
#include "1.8 tft_display.h"

extern float linear_ccd_buffer[128];


#define CLK_PORT GPIOA
#define CLK_PIN GPIO_Pin_1

#define SI_PORT GPIOA
#define SI_PIN GPIO_Pin_0

#define AO_channel 10



void linear_ccd_init();
void linear_ccd_read();
