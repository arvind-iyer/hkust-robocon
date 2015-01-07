#include "debug.h"

u8 buzzer_on = 0;
u16 buzzer_period = 0;
u8 buzzer_count = 0;

/**
  * @brief  Initialization of buzzer
  * @param  None
  * @retval None
  */
void buzzer_init(void)
{		   	
	GPIO_InitTypeDef BUZZER_InitStructure; 	
	NVIC_InitTypeDef NVIC_InitStructure;		  		
	RCC_APB2PeriphClockCmd(BUZZER_RCC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	BUZZER_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			   
	BUZZER_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	BUZZER_InitStructure.GPIO_Pin = BUZZER_PIN; 
	GPIO_Init(BUZZER_PORT, &BUZZER_InitStructure); 
	
	TIM2->ARR = 1000;
	TIM2->PSC = 7199;
	TIM2->EGR = 1;
	TIM2->SR = 0;
	TIM2->DIER = 1;
	TIM2->CR1 = 1;
	TIM_Cmd(TIM2, DISABLE);
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;   
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Timer2 for counting the buzzer
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	
	if (!buzzer_on) {
		GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
		TIM_Cmd(TIM2, DISABLE);
		return;	
	}
	
	if ((buzzer_count--)%2 == 0)
		GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
	else
		GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);

	if (buzzer_count == 0)
		buzzer_on = 0;
}

/**
  * @brief  Generate specific pattern of buzzer
  * @param  count: number of buzz to be generated
  * @param  period: time for each buzz (per 10 ms)
  * @retval None
  */
void buzzer_control(u8 count, u16 period)
{
	buzzer_count = count*2-1;
	buzzer_on = 1;
	TIM_SetAutoreload(TIM2, period*100-1);
	TIM_SetCounter(TIM2, 0);
	GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
	TIM_Cmd(TIM2, ENABLE);
}

/**
  * @brief  Initialaztion of LED
  * @param  None
  * @retval None
  */
void led_init(void)
{
	GPIO_InitTypeDef LED_InitStructure;
  
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOG, ENABLE); 

	LED_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;			   
  	LED_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	LED_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 			    				
  	GPIO_Init(GPIOG, &LED_InitStructure);

  	LED_InitStructure.GPIO_Pin = GPIO_Pin_7; 			    				
  	GPIO_Init(GPIOD, &LED_InitStructure);
	
	led_control(LED_R | LED_G | LED_B, LED_OFF);
}

/**
  * @brief  Turn on/off the LED
  * @param  led: which LEDs to be turned on/off (LED_R, LED_G, LED_B)
  * @param  state: turn on/off (LED_ON, LED_OFF)
  * @retval None
  */
void led_control(u8 led, u8 state)
{
	if (led & LED_R) {
		if (state)
			GPIO_ResetBits(GPIOG, GPIO_Pin_6);
		else  
			GPIO_SetBits(GPIOG, GPIO_Pin_6);
	}
	if (led & LED_G) {
		if (state)
			GPIO_ResetBits(GPIOG, GPIO_Pin_7);
		else  
			GPIO_SetBits(GPIOG, GPIO_Pin_7);
	}
	if (led & LED_B) {
		if (state)
			GPIO_ResetBits(GPIOD, GPIO_Pin_7);
		else  
			GPIO_SetBits(GPIOD, GPIO_Pin_7);
	}
}
