#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "approx_math.h"

#define MAX_PWM									1799
#define MOTOR_TIM								TIM3
#define MOTOR_TIM_RCC_init()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)

#define MOTOR_MAG_GPIOx					GPIOB
#define MOTOR_MAG_Pin						GPIO_Pin_0

#define MOTOR_DIR_GPIOx					GPIOC
#define MOTOR_DIR1_Pin					GPIO_Pin_4
#define MOTOR_DIR2_Pin					GPIO_Pin_5

#define MOTOR_MAG_RCC_init()		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE)
#define MOTOR_DIR_RCC_init()		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE)

typedef enum {
	DONT_CARE = -1,
	ANTI_CKW = 0,
	CKW = 1
} DIRECTION;
 
void motor_init(void);
void motion_set_motor(s32 compare);
void motor_set_pwm(s32 pwm);

extern bool FULL_SPEED_LIMIT;

#endif	// MOTOR_H
