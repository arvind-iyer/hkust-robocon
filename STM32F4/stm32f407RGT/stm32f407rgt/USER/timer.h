
#include "stm32f4xx_tim.h"
#include "stm32f4xx.h"
#include "gpio.h"









//void (* const Channel_init[4]) (TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct) = {TIM_OC1Init, TIM_OC2Init, TIM_OC3Init, TIM_OC4Init};
//void (* const Channel_config[4]) (TIM_TypeDef* TIMx, uint16_t TIM_OCPreload) = {TIM_OC1PreloadConfig, TIM_OC2PreloadConfig, TIM_OC3PreloadConfig, TIM_OC4PreloadConfig};





void timer_rcc_init(TIM_TypeDef* TIMx);
void timer_init(TIM_TypeDef* TIMx, u16 Prescaler, u16 CounterMode, u16 Peroid, u16 ClockDivision);
void pwm_timer_init(u16 OC,TIM_TypeDef *TIMER, u16 OCMode, u16 OutputState, u16 Pulse, u16 OCPolarity);
void encoder_timer_init(TIM_TypeDef* TIMx, const u16 initial_value);

