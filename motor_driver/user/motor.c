/**
  ******************************************************************************
  * @file    motor.c
  * @author  William LEE
  * @version V3.5.0
  * @date    24-January-2015
  * @brief   This file provides the use of motor (pwm and direction).
  ******************************************************************************
  * @attention
  *
	*
  ******************************************************************************
  */
#include "motor.h"

bool FULL_SPEED_LIMIT = false;		// global flag indicate whether exceeds limit.

/**
	* @brief 		Motor initialization
  * @param	  None
  * @retval 	None
	*/
void motor_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* PWM magnitude GPIO configuration */
	MOTOR_MAG_RCC_init();
	GPIO_InitStructure.GPIO_Pin = MOTOR_MAG_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(MOTOR_MAG_GPIOx, &GPIO_InitStructure);

	/* PWM direction GPIO configuation */
	MOTOR_DIR_RCC_init();
	GPIO_InitStructure.GPIO_Pin = MOTOR_DIR1_Pin | MOTOR_DIR2_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(MOTOR_DIR_GPIOx, &GPIO_InitStructure);
	
	/* Time base configuration */
	MOTOR_TIM_RCC_init();
	TIM_TimeBaseStructure.TIM_Period = MAX_PWM;  // 40KHZ for motor (72M / 1.8k = 40k)
	TIM_TimeBaseStructure.TIM_ClockDivision =  TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(MOTOR_TIM, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(MOTOR_TIM, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(MOTOR_TIM, ENABLE);

	/* TIM enable counter */
	TIM_Cmd(MOTOR_TIM, ENABLE);
	TIM_CtrlPWMOutputs(MOTOR_TIM, ENABLE);
	// Open loop 0 signal for initial state.
	motor_set_pwm(0);
}

/**
	* @brief Set motor direction (private)
	* @param dir: CKW (clokwise) or ANTI_CKW, or DONT_CARE (use previous direction)
  * @retval None
	*/
static void set_dir(DIRECTION dir)
{
	if (dir == ANTI_CKW) {
		GPIO_SetBits(MOTOR_DIR_GPIOx, MOTOR_DIR1_Pin);
		GPIO_ResetBits(MOTOR_DIR_GPIOx, MOTOR_DIR2_Pin);
	} else if (dir == CKW) {
		GPIO_SetBits(MOTOR_DIR_GPIOx,MOTOR_DIR2_Pin);
		GPIO_ResetBits(MOTOR_DIR_GPIOx,MOTOR_DIR1_Pin);
	}
}

/**
	* @brief Set pwm directly to the motor
	* @param pwm: the pwm value ranging from
								-1799 (anticlockwise full power) to 1799 (clockwise full power),
								0 is stop.
  * @retval None
	*/
void motor_set_pwm(s32 pwm)
{
	if (pwm > MAX_PWM) {
		pwm = MAX_PWM;
		FULL_SPEED_LIMIT = true;
	} else if (pwm < -MAX_PWM) {
		pwm = -MAX_PWM;
		FULL_SPEED_LIMIT = true;
	} else {
		FULL_SPEED_LIMIT = false;
	}
	
	// Set pwm 0 is FULL POWER.
	TIM_SetCompare3(MOTOR_TIM, MAX_PWM - Abs(pwm));
	// Dont care the direction if pwm is 0.
	set_dir(pwm > 0 ? CKW : (pwm < 0 ? ANTI_CKW : DONT_CARE));
}
