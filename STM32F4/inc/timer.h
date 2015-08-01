#ifndef	TIMER_H
#define	TIMER_H

#include "stm32f4xx_tim.h"
#include "stm32f4xx.h"
#include "gpio.h"

enum Channel{
	Ch1 = 0,
	Ch2,
	Ch3,
	Ch4
};

class TIMER {
public:
	TIMER(TIM_TypeDef* const TIMn, Channel Chn);
	void AF_init(GPIO* gpio_of_TIM);
	void SetCompare(uint32_t Compare_value);

	TIM_TypeDef* const TIMx;
	Channel Chx;
};

void (* const Channel_init[4]) (TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct) = {TIM_OC1Init, TIM_OC2Init, TIM_OC3Init, TIM_OC4Init};
void (* const Channel_config[4]) (TIM_TypeDef* TIMx, uint16_t TIM_OCPreload) = {TIM_OC1PreloadConfig, TIM_OC2PreloadConfig, TIM_OC3PreloadConfig, TIM_OC4PreloadConfig};

extern TIMER
	TIM1Ch1, TIM1Ch2, TIM1Ch3, TIM1Ch4,
	TIM2Ch1, TIM2Ch2, TIM2Ch3, TIM2Ch4,
	TIM3Ch1, TIM3Ch2, TIM3Ch3, TIM3Ch4,
	TIM4Ch1, TIM4Ch2, TIM4Ch3, TIM4Ch4,
	TIM5Ch1, TIM5Ch2, TIM5Ch3, TIM5Ch4,
	TIM6Ch1, TIM6Ch2, TIM6Ch3, TIM6Ch4,
	TIM7Ch1, TIM7Ch2, TIM7Ch3, TIM7Ch4,
	TIM8Ch1, TIM8Ch2, TIM8Ch3, TIM8Ch4,
	TIM9Ch1, TIM9Ch2, TIM9Ch3, TIM9Ch4,
	TIM10Ch1, TIM10Ch2, TIM10Ch3, TIM10Ch4,
	TIM11Ch1, TIM11Ch2, TIM11Ch3, TIM11Ch4,
	TIM12Ch1, TIM12Ch2, TIM12Ch3, TIM12Ch4,
	TIM13Ch1, TIM13Ch2, TIM13Ch3, TIM13Ch4,
	TIM14Ch1, TIM14Ch2, TIM14Ch3, TIM14Ch4,
	TIM15Ch1, TIM15Ch2, TIM15Ch3, TIM15Ch4,
	TIM16Ch1, TIM16Ch2, TIM16Ch3, TIM16Ch4,
	TIM17Ch1, TIM17Ch2, TIM17Ch3, TIM17Ch4;




void timer_rcc_init(TIM_TypeDef* TIMx);
void timer_init(TIM_TypeDef* TIMx, u16 Prescaler, u16 CounterMode, u16 Peroid, u16 ClockDivision);
void pwm_timer_init(TIMER TIM, u16 OCMode, u16 OutputState, u16 Pulse, u16 OCPolarity);
void encoder_timer_init(TIM_TypeDef* TIMx, const u16 initial_value);

#endif /* TIMER_H */
