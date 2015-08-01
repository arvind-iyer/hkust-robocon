#include "timer.h"

TIMER::TIMER(TIM_TypeDef* const TIM, Channel ch) : TIMx(TIM), Chx(ch) {}


TIMER
	TIM1Ch1(TIM1, Ch1),
	TIM1Ch2(TIM1, Ch2),
	TIM1Ch3(TIM1, Ch3),
	TIM1Ch4(TIM1, Ch4),

	TIM2Ch1(TIM2, Ch1),
	TIM2Ch2(TIM2, Ch2),
	TIM2Ch3(TIM2, Ch3),
	TIM2Ch4(TIM2, Ch4),

	TIM3Ch1(TIM3, Ch1),
	TIM3Ch2(TIM3, Ch2),
	TIM3Ch3(TIM3, Ch3),
	TIM3Ch4(TIM3, Ch4),

	TIM4Ch1(TIM4, Ch1),
	TIM4Ch2(TIM4, Ch2),
	TIM4Ch3(TIM4, Ch3),
	TIM4Ch4(TIM4, Ch4),

	TIM5Ch1(TIM5, Ch1),
	TIM5Ch2(TIM5, Ch2),
	TIM5Ch3(TIM5, Ch3),
	TIM5Ch4(TIM5, Ch4),

	TIM6Ch1(TIM6, Ch1),
	TIM6Ch2(TIM6, Ch2),
	TIM6Ch3(TIM6, Ch3),
	TIM6Ch4(TIM6, Ch4),

	TIM7Ch1(TIM7, Ch1),
	TIM7Ch2(TIM7, Ch2),
	TIM7Ch3(TIM7, Ch3),
	TIM7Ch4(TIM7, Ch4),

	TIM8Ch1(TIM8, Ch1),
	TIM8Ch2(TIM8, Ch2),
	TIM8Ch3(TIM8, Ch3),
	TIM8Ch4(TIM8, Ch4),

	TIM9Ch1(TIM9, Ch1),
	TIM9Ch2(TIM9, Ch2),
	TIM9Ch3(TIM9, Ch3),
	TIM9Ch4(TIM9, Ch4),

	TIM10Ch1(TIM10, Ch1),
	TIM10Ch2(TIM10, Ch2),
	TIM10Ch3(TIM10, Ch3),
	TIM10Ch4(TIM10, Ch4),

	TIM11Ch1(TIM11, Ch1),
	TIM11Ch2(TIM11, Ch2),
	TIM11Ch3(TIM11, Ch3),
	TIM11Ch4(TIM11, Ch4),

	TIM12Ch1(TIM12, Ch1),
	TIM12Ch2(TIM12, Ch2),
	TIM12Ch3(TIM12, Ch3),
	TIM12Ch4(TIM12, Ch4),

	TIM13Ch1(TIM13, Ch1),
	TIM13Ch2(TIM13, Ch2),
	TIM13Ch3(TIM13, Ch3),
	TIM13Ch4(TIM13, Ch4),

	TIM14Ch1(TIM14, Ch1),
	TIM14Ch2(TIM14, Ch2),
	TIM14Ch3(TIM14, Ch3),
	TIM14Ch4(TIM14, Ch4);

void timer_init(TIM_TypeDef* TIMx, u16 Prescaler, u16 CounterMode, u16 Peroid, u16 ClockDivision)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	timer_rcc_init(TIMx);
	TIM_TimeBaseStructure.TIM_Period = Peroid;
	TIM_TimeBaseStructure.TIM_ClockDivision =  ClockDivision;
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;
	TIM_TimeBaseStructure.TIM_CounterMode = CounterMode;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
}

void timer_rcc_init(TIM_TypeDef* TIMx)
{
	if (TIMx == TIM1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	} else if (TIMx == TIM2) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	} else if (TIMx == TIM3) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	} else if (TIMx == TIM4) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	} else if (TIMx == TIM5) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	} else if (TIMx == TIM6) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	} else if (TIMx == TIM7) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	} else if (TIMx == TIM8) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	} else if (TIMx == TIM9) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	} else if (TIMx == TIM10) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	} else if (TIMx == TIM11) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
	} else if (TIMx == TIM12) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	} else if (TIMx == TIM13) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	} else if (TIMx == TIM14) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	}
}

void pwm_timer_init(TIMER TIM, u16 OCMode, u16 OutputState, u16 Pulse, u16 OCPolarity)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCPolarity = OCPolarity;   		// set "high" to be effective output
	TIM_OCInitStructure.TIM_OCMode = OCMode;	           		// produce output when counter < CCR
	TIM_OCInitStructure.TIM_OutputState = OutputState;  	// this part enable the output
	TIM_OCInitStructure.TIM_Pulse = Pulse;
	
	if (TIM.TIMx == TIM1 || TIM.TIMx == TIM8) {
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;   		
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;	
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable; 
	}
	Channel_init[TIM.Chx](TIM.TIMx, &TIM_OCInitStructure);
	Channel_config[TIM.Chx](TIM.TIMx, TIM_OCPreload_Enable);
	TIM_CtrlPWMOutputs(TIM.TIMx, ENABLE);
	TIM_ARRPreloadConfig(TIM.TIMx, ENABLE);
  TIM_Cmd(TIM.TIMx, ENABLE);	
}

void encoder_timer_init(TIM_TypeDef* TIMx, const u16 initial_value)
{
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12,						//TIM_EncoderMode_TI12->count on 4 edge per cycle
	                         TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//	TIM_ICStructInit(&TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_ICFilter = 8;
//	TIM_ICInit(TIMx, &TIM_ICInitStructure);
//	
//	// Clear all pending interrupts
//	TIM_ClearFlag(TIMx, TIM_FLAG_Update);
//	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
	// Reset counter to initial value
	TIM_SetCounter(TIMx, initial_value);
	// Counter Enable
	TIM_Cmd(TIMx, ENABLE);
}

void TIMER::AF_init(GPIO* gpio_of_TIM)
{
	// For F4 AF init, see stm32f4xx_gpio.h starting from line 243.
	if (TIMx == TIM1 || TIMx == TIM2) {
		// AF1 for TIM1 and 2
		GPIO_PinAFConfig(gpio_of_TIM->gpio, gpio_of_TIM->get_pin_source(), GPIO_AF_TIM1);
	} else if (TIMx == TIM3 || TIMx == TIM4 || TIMx == TIM5) {
		// AF2 for TIM3 to 5
		GPIO_PinAFConfig(gpio_of_TIM->gpio, gpio_of_TIM->get_pin_source(), GPIO_AF_TIM3);
	}	else if (TIMx == TIM8 || TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11) {
		// AF3 for TIM 8 to 11
		GPIO_PinAFConfig(gpio_of_TIM->gpio, gpio_of_TIM->get_pin_source(), GPIO_AF_TIM8);
	}	else if (TIMx == TIM12 || TIMx == TIM13 || TIMx == TIM14) {
		// AF9 for TIM 12 to 14
		GPIO_PinAFConfig(gpio_of_TIM->gpio, gpio_of_TIM->get_pin_source(), GPIO_AF_TIM12);
	}
}
