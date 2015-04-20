#ifndef	TIMER_H
#define	TIMER_H

#include "stm32f4xx_tim.h"
#include "stm32f4xx.h"

enum Channel{
	Ch1 = 0,
	Ch2,
	Ch3,
	Ch4
};

struct TIMER{
	TIM_TypeDef* TIMx;
	Channel Chx;
};

//class TIMER_ {
//public:
//	TIMER_();
//	void output_pwm(uint16_t);
//
//private:
//	TIM_TypeDef* const TIMx;
//	const Channel Chx;
//	void (* const Set_pwm) (TIM_TypeDef*, uint16_t);
//};

void (* const Channel_init[4]) (TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct) = {TIM_OC1Init, TIM_OC2Init, TIM_OC3Init, TIM_OC4Init};
void (* const Channel_config[4]) (TIM_TypeDef* TIMx, uint16_t TIM_OCPreload) = {TIM_OC1PreloadConfig, TIM_OC2PreloadConfig, TIM_OC3PreloadConfig, TIM_OC4PreloadConfig};


void timer_rcc_init(TIM_TypeDef* TIMx);
void timer_init(TIM_TypeDef* TIMx, u16 Prescaler, u16 CounterMode, u16 Peroid, u16 ClockDivision);
void pwm_timer_init(TIMER TIM, u16 OCMode, u16 OutputState, u16 Pulse, u16 OCPolarity);
void encoder_timer_init(TIM_TypeDef* TIMx, const u16 initial_value);

#endif /* TIMER_H */
