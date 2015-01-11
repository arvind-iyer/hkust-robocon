#ifndef _ENCODER_H_
#define _ENCODER_H_

// include ic library
#include "stm32f10x.h"
#include "stm32f10x_tim.h"

typedef struct {
	TIM_TypeDef* timer;
	u32 clock_source;
	u32 GPIO_clock_source;
	u16 port1;
	u16 port2;
	GPIO_TypeDef* GPIOx;
} Encoder_Typedef[];

// define
#define ENCODER_NO												2
// Encoder 1 details
#define ENCODER_TIMER1										TIM4
#define ENCODER_TIMER1_CLOCK_SOURCE				RCC_APB1Periph_TIM4						// in APB 1
#define ENCODER_TIMER1_GPIO_CLOCK_SOURCE	RCC_APB2Periph_GPIOB					// in APB 2
#define ENCODER_TIMER1_PORT1							GPIO_Pin_6
#define ENCODER_TIMER1_PORT2							GPIO_Pin_7
#define ENCODER_TIMER1_GPIOx							GPIOB
// Encoder 2 details
#define ENCODER_TIMER2										TIM5
#define ENCODER_TIMER2_CLOCK_SOURCE				RCC_APB1Periph_TIM5						// in APB 1
#define ENCODER_TIMER2_GPIO_CLOCK_SOURCE	RCC_APB2Periph_GPIOA					// in APB 2
#define ENCODER_TIMER2_PORT1							GPIO_Pin_0
#define ENCODER_TIMER2_PORT2							GPIO_Pin_1
#define ENCODER_TIMER2_GPIOx							GPIOA

#define ENCODER_MAX_CHANGE								20000

typedef enum {
	ENCODER1 = 0,
	ENCODER2 = 1
} ENCODER;

// function declaration
void encoder_init(void);
u32 get_count(ENCODER ENCODERx);

#endif
