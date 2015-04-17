#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "gpio.h"

#define ULTRASONIC_TIM							TIM9
#define ULTRASONIC_RCC							RCC_APB2Periph_TIM9
#define ULTRASONIC_IRQn						  TIM1_BRK_TIM9_IRQn
#define ULTRASONIC_IRQHandler			  void TIM1_BRK_TIM9_IRQHandler(void)

#define ULTRASONIC_TRIG_GPIO        ((GPIO*) &PE0)
#define ULTRASONIC_ECHO_GPIO        ((GPIO*) &PE1)

#define ULTRASONIC_TRIG_PULSE         10    // 10 us
#define ULTRASONIC_ECHO_PULSE_COUNT   5

void ultrasonic_init(void);
u32 get_pulse_width(void);
u32 get_distance(void);
u32 ultrasonic_get_distance_avg(void);
u32 ultrasonic_get_count(void);
u32 ultrasonic_get_successful_count(void);


#endif /* __ULTRASONIC_H */
