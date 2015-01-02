#include "buzzer.h"


u8 buzzer_on_flag = 0;
u16 buzzer_period = 0;
u16 buzzer_time_ms = 0;
u8 buzzer_count = 0;

/**
  * @brief  Initialization of buzzer
  * @param  None
  * @retval None
  */
void buzzer_init(void)
{		   	
	GPIO_InitTypeDef BUZZER_InitStructure; 			
	RCC_APB2PeriphClockCmd(BUZZER_RCC, ENABLE);
	BUZZER_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			   
	BUZZER_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	BUZZER_InitStructure.GPIO_Pin = BUZZER_PIN;
	
	
	// buzzer frequency init
	/*
	{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      // TimeBase is for timer setting   > refer to P. 344 of library
		TIM_OCInitTypeDef  TIM_OCInitStructure;             // OC is for channel setting within a timer  > refer to P. 342 of library

		
		RCC_APB1PeriphClockCmd(BUZZER_TIM_RCC, ENABLE);
		//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		GPIO_PinRemapConfig(BUZZER_TIM_REMAP, ENABLE);
		
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   // counter will count up (from 0 to FFFF)
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;       //timer clock = dead-time and sampling clock 	
		TIM_TimeBaseStructure.TIM_Prescaler = 7100;                         // 1MHz
		TIM_TimeBaseStructure.TIM_Period = 500;	                       	//	

		
		TIM_TimeBaseInit(BUZZER_TIM, &TIM_TimeBaseStructure);       // this part feeds the parameter we set above
		
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;         //set "high" to be effective output
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	             //produce output when counter < CCR
		
		TIM_OCInitStructure.TIM_Pulse = 250;
		
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
		TIM_ARRPreloadConfig(BUZZER_TIM, ENABLE);
		TIM_Cmd(BUZZER_TIM, ENABLE);	
		
		TIM_OC1Init(BUZZER_TIM, &TIM_OCInitStructure);
		TIM_OC2Init(BUZZER_TIM, &TIM_OCInitStructure);
		TIM_OC3Init(BUZZER_TIM, &TIM_OCInitStructure);
		TIM_OC4Init(BUZZER_TIM, &TIM_OCInitStructure);
	
	
	}
	*/
	
	GPIO_Init(BUZZER_PORT, &BUZZER_InitStructure); 
	buzzer_off();
	

}

void buzzer_on(void)
{
	GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
}

void buzzer_off(void)
{
	GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
}

/**
  * @brief  Buzzer check per tick
  * @param  None
  * @retval None
  */
void buzzer_check(void)
{
	if (buzzer_on_flag == 0 && buzzer_count == 0) {return;}	 // Do nothing
	--buzzer_time_ms;
	if (buzzer_time_ms == 0) {
		buzzer_on_flag = !buzzer_on_flag;
		buzzer_on_flag ? buzzer_on() : buzzer_off();

		buzzer_time_ms = buzzer_period;
		
		if (!buzzer_on_flag) {
			--buzzer_count;
		}
	}
}

/**
  * @brief  Generate specific pattern of buzzer
  * @param  count: number of buzz to be generated
  * @param  period: time for each buzz (ms)
  * @retval None
  */
void buzzer_control(u8 count, u16 period)
{
	if (count == 0 || period == 0) {return;}	// Do nothing

	buzzer_count = count;
	buzzer_period = buzzer_time_ms = period;
	buzzer_on_flag = 1;
	buzzer_on();
}
