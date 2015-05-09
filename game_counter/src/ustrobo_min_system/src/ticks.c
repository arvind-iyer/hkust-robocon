#include "ticks.h"
#include "flash.h"

volatile u16 ticks = 0;
volatile u16 seconds = 0;
volatile u32 current_time = 0;
static u16 days_left = 0;

/**
  * @brief  Get the ticks passed from 0-999
  * @param  None
  * @retval ticks passed
  */
u16 get_ticks(void) {
	return ticks;
}

/**
  * @brief  Get the seconds passed from
  * @param  seconds
  * @retval ticks passed
  */
u16 get_seconds(void) {
	return seconds;
}

void set_current_time(u32 time) 
{
  current_time = time;
}

u32 get_current_time(void)
{
  return current_time;
}

u32 get_full_ticks(void)
{
	return seconds * 1000 + ticks;
}

void set_days_left(u16 days) 
{
	days_left = days;
	write_flash(0, (s32) days_left); 
}

u16 get_days_left(void)
{
	return days_left;
}

/**
  * @brief  Initialization of ticks timer
  * @param  None
  * @retval None
  */
void ticks_init(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      									// TimeBase is for timer setting   > refer to P. 344 of library

	RCC_APB1PeriphClockCmd(TICKS_RCC , ENABLE);
	
//	TICKS_TIM->PSC = SystemCoreClock / 1000000 - 1;		// Prescaler
//	TICKS_TIM->ARR = 1000;
//	TICKS_TIM->EGR = 1;
//	TICKS_TIM->SR = 0;
//	TICKS_TIM->DIER = 1;
//	TICKS_TIM->CR1 = 1;
	
	TIM_TimeBaseStructure.TIM_Period = 1000;	                 				       // Timer period, 1000 ticks in one second
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1;     // 72M/1M - 1 = 71
	TIM_TimeBaseInit(TICKS_TIM, &TIM_TimeBaseStructure);      							 // this part feeds the parameter we set above
	
	TIM_ClearITPendingBit(TICKS_TIM, TIM_IT_Update);												 // Clear Interrupt bits
	TIM_ITConfig(TICKS_TIM, TIM_IT_Update, ENABLE);													 // Enable TIM Interrupt
	TIM_Cmd(TICKS_TIM, ENABLE);																							 // Counter Enable

	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TICKS_IRQn;
	NVIC_Init(&NVIC_InitStructure);
  NVIC_SetPriority(TICKS_IRQn, 0);
	
	//SysTick_Config(SystemCoreClock/1000);
	ticks = seconds = 0;
	
	u16 tmp = (u16) read_flash(0);
	
	if (tmp > 0 && tmp < 999) {
		days_left = tmp;
	}
	
}

static void new_date(void)
{
	if (days_left > 0) {
		--days_left;
	}
	write_flash(0, days_left);
}

/**
  * @brief  Timer for ticks
  * @param  None
  * @retval None
  */
TICKS_IRQHandler
{
  if (TIM_GetITStatus(TICKS_TIM, TIM_IT_Update) != RESET) {
    TIM_ClearFlag(TICKS_TIM, TIM_FLAG_Update);
    //TIM_ClearITPendingBit(TICKS_TIM, TIM_IT_Update);

    if (ticks >= 999) {
      ticks = 0;
      seconds++;
      if (current_time >= 24 * 60 * 60 - 1) {
        current_time = 0;
				new_date();
      } else {
        ++current_time;
      }
      
    } else {
      ticks++;
    }

    buzzer_check();
  }
	
}

