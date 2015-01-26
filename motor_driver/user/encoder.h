#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f10x.h"

#define ENCODER_TIM												TIM2
#define ENCODER_TIM_PORT1									GPIO_Pin_0
#define ENCODER_TIM_PORT2									GPIO_Pin_1
#define ENCODER_TIM_GPIOx									GPIOA

#define encoder_gpio_rcc_init()						RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)
#define encoder_rcc_init()								RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE)
#define SOURCE_ENCODER_INIT_VAL						32768

void encoder_init(void);
void encoder_update(void);
s32 get_encoder(void);
s32 get_encoder_vel(void);

#endif	// ENCODER_H
