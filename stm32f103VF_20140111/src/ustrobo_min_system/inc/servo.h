#ifndef _SERVO_H_
#define _SERVO_H_

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"	  
#include "stm32f10x_tim.h"

typedef struct {
	u16 servo_tim_ch;
	u16 servo_pin;
	FunctionalState state;
	void (* oc_init_function)(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
} SERVO_PWM_STRUCT[];

#define SERVO1					0
#define SERVO2					1
#define SERVO3					2
#define SERVO4					3
#define SERVO_TIM				TIM8
#define SERVO_PORT			GPIOC
#define SERVO_TIM_RCC		RCC_APB2Periph_TIM8
#define SERVO_GPIO_RCC	RCC_APB2Periph_GPIOC

void servo_init(void);
void servo_control(u8 servo_id , u16 val);   // value from 0 to 1000

#endif		/*  _SERVO_H_ */
