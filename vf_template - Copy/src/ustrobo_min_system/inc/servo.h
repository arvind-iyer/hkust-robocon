#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#define MENU_ADD_SERVO_TEST menu_add(6, "Servo Test", servo_test)

typedef struct {
	u16 servo_tim_ch;
	u16 servo_pin;
	FunctionalState state;
	void (* oc_init_function)(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
} SERVO_PWM_STRUCT[];

#define SERVO1	0
#define SERVO2	1
#define SERVO3	2
#define SERVO4	3

#define SERVO_TIM		TIM4
#define SERVO_PORT		GPIOD
#define SERVO_TIM_RCC	RCC_APB1Periph_TIM4
#define SERVO_GPIO_RCC	RCC_APB2Periph_GPIOD	

void servo_init(void);
void servo_control(u8 servo_id , u16 val);   // value from 0 to 1000

#endif		/*  __SERVO_H */
