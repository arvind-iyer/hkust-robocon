/**
  ******************************************************************************
  * @file    encoder.c
  * @author  William LEE
  * @version V3.5.0
  * @date    24-January-2015
  * @brief   This file provides encoder data and change of encoder.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "encoder.h"

static s32 adjusted_encoder = 0;		// Encoder value sent to main board, 8 bytes.
static s32 rate_change_encoder = 0;	// differentiation of encoder, which means actual velocity.

/**
	* @brief	Encoder initialization
	*	@param	None.
	*	@retval	None.
	*/
void encoder_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef encoder_TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	/* GPIOA Configuration: TIM2 ch1, ch2, input float*/
	encoder_gpio_rcc_init();
	
	GPIO_InitStructure.GPIO_Pin = ENCODER_TIM_PORT1 | ENCODER_TIM_PORT2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENCODER_TIM_GPIOx, &GPIO_InitStructure);

	/* Timer configuration in Encoder mode */
	encoder_rcc_init();
	
	encoder_TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling
	encoder_TIM_TimeBaseStructure.TIM_Period = 65535;
	encoder_TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	encoder_TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(ENCODER_TIM, &encoder_TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(ENCODER_TIM, TIM_EncoderMode_TI12,						//TIM_EncoderMode_TI12->count on 4 edge per cycle
	                         TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 8;
	TIM_ICInit(ENCODER_TIM, &TIM_ICInitStructure);
	
	// Clear all pending interrupts
	TIM_ClearFlag(ENCODER_TIM, TIM_FLAG_Update);
	TIM_ITConfig(ENCODER_TIM, TIM_IT_Update, ENABLE);
	// Reset counter to initial value
	TIM_SetCounter(ENCODER_TIM, SOURCE_ENCODER_INIT_VAL);
	// Counter Enable
	TIM_Cmd(ENCODER_TIM, ENABLE);
}

/**
	* @brief Update the encoder value and real velocity
	*	@param	None.
	*	@retval	None.
	*/
void encoder_update(void)
{
	static s32 flowing_flag = 0;						// -ve: no. of underflow, +ve: no. of overflow
	static s32 prev_adjusted_encoder = 0;		// record data of past
	
	s32 source_encoder = TIM_GetCounter(ENCODER_TIM);	// value directly from timer count
	if (source_encoder > 62768) {
		// Treat as underflow, add 30000 to timer counter.
		TIM_SetCounter(ENCODER_TIM, source_encoder - 30000);
		++flowing_flag;
	} else if (source_encoder < 2768) {
		// Treat as underflow, add 30000 to timer counter.
		TIM_SetCounter(ENCODER_TIM, source_encoder + 30000);
		--flowing_flag;
	}

	adjusted_encoder = flowing_flag * 30000 + TIM_GetCounter(ENCODER_TIM) - SOURCE_ENCODER_INIT_VAL;

	rate_change_encoder = -adjusted_encoder + prev_adjusted_encoder;
	prev_adjusted_encoder = adjusted_encoder;
}

/** @brief 		Get encoder value outside this file
	*	@param	  None.
	* @retval   The encoder value
	* @example  Used in sending encoder value to mainboard
	*/
s32 get_encoder(void)
{
	return adjusted_encoder;
}

/** @brief 	Get encoder velocity (rate change of encoder) outside this file
	*	@param	None.
	* @retval The rate of change of encoder value (velocity)
	*/
s32 get_encoder_vel(void)
{
	return rate_change_encoder;
}

