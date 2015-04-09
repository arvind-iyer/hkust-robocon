#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

#define ADC_COUNT                     3
#define ADC_CHANNEL_COUNT             18

#define ADC_CHANNEL_UNUSED_VALUE      ((u32) -1)
void adc_init(void);
u32 get_adc_value(u8 channel);

#endif  /** __ADC_H **/
