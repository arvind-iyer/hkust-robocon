
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"

void ADC1_init(void);
float get_battery();
u16 raw_data();
float get_temperature();
u16 get_temp_level();