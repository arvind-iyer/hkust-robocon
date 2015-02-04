#ifndef _SERVO_H_
#define _SERVO_H_

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"	  
#include "stm32f10x_tim.h"

#define SERVO_COUNT     4
#define SERVO_TIM				TIM8
#define SERVO_PORT			GPIOC
#define SERVO_TIM_RCC		RCC_APB2Periph_TIM8
#define SERVO_GPIO_RCC	RCC_APB2Periph_GPIOC

typedef struct {
	u16 servo_tim_ch;
	u16 servo_pin;
	FunctionalState state;
	void (*oc_init_function)(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
  void (*TIM_SetCompare) (TIM_TypeDef* TIMx, uint16_t Compare1);
} SERVO_PWM_STRUCT[];

typedef enum {
  SERVO1,
  SERVO2,
  SERVO3,
  SERVO4
} SERVO_ID;



void servo_init(void);
void servo_control(SERVO_ID servo_id , u16 val);   // value from 0 to 1000

#endif		/*  _SERVO_H_ */
