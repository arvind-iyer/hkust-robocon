#include "debug.h"

u8 buzzer_on = 0;
u16 buzzer_period = 0;
u32 buzzer_count = 0;
u16 buzzer_count2 = 0;

/**
  * @brief  Initialization of buzzer
  * @param  None
  * @retval None
  */
void buzzer_init(void)
{		   	
	GPIO_InitTypeDef BUZZER_InitStructure;
	RCC_APB2PeriphClockCmd(BUZZER_RCC, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	BUZZER_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	BUZZER_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	BUZZER_InitStructure.GPIO_Pin = BUZZER_PIN;
	GPIO_Init(BUZZER_PORT, &BUZZER_InitStructure);
	
	GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
}

/**
  * @brief  Generate specific pattern of buzzer
  * @param  count: number of buzz to be generated
  * @param  period: time for each buzz (per 10 ms)
  * @retval None
  */
void buzzer_control(u8 count, u16 period)
{
	buzzer_on = 1;
	buzzer_period = period*10;
	buzzer_count = count*2*buzzer_period-1;
	buzzer_count2 = count*2-1;
	GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
}

void buzzer_update(void)
{
	if ((buzzer_count2--) % 2 == 0)
		GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
	else
		GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
	
	if (buzzer_count2 == 0) {
		buzzer_on = 0;
		GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
	}
}
