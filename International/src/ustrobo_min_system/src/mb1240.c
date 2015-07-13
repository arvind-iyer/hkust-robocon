#include "mb1240.h"

static u32 mb1240_length = 0;
static u32 mb1240_last_length = 0;
static u8 mb1240_pulse_on_flag = 0;

static u16 mb1240_count[MB1240_RATE_COUNT] = {0};
static u16 last_second = (u16)-1;
static u16 mb1240_rate = 0;
void mb1240_init(void)
{
  // GPIO init
  gpio_init(MB1240_GPIO, GPIO_Speed_50MHz, GPIO_Mode_IPD, 1);
  
 	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      									// TimeBase is for timer setting   > refer to P. 344 of library
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  EXTI_InitTypeDef   EXTI_InitStructure;
  
	RCC_APB2PeriphClockCmd(MB1240_RCC , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	ULTRASONIC_TIM->PSC = SystemCoreClock / 100000 - 1;		// Prescaler
//	ULTRASONIC_TIM->ARR = 1;
//	ULTRASONIC_TIM->EGR = 1;
//	ULTRASONIC_TIM->SR = 0;
//	ULTRASONIC_TIM->DIER = 1;
//	ULTRASONIC_TIM->CR1 = 1;
	
	TIM_TimeBaseStructure.TIM_Period = 58;	                 				         // 58 us
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1;     // 72M/1M - 1 = 71
	TIM_TimeBaseInit(MB1240_TIM, &TIM_TimeBaseStructure);      							 // this part feeds the parameter we set above


    
	TIM_ClearITPendingBit(MB1240_TIM, TIM_IT_Update);												 // Clear Interrupt bits
	TIM_ITConfig(MB1240_TIM, TIM_IT_Update, ENABLE);													 // Enable TIM Interrupt
  //TIM_ITConfig(MB1240_TIM, TIM_IT_CC1, ENABLE);													 // Enable TIM Interrupt
	TIM_Cmd(MB1240_TIM, ENABLE);																							 // Counter Enable
  TIM_SetCounter(MB1240_TIM, 0);


  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource0);
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
   
  TIM_ARRPreloadConfig(MB1240_TIM, ENABLE);
  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = MB1240_IRQn;
	NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Init(&NVIC_InitStructure); 
}


void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
    EXTI_ClearITPendingBit(EXTI_Line0);
    
    if (gpio_read_input(MB1240_GPIO)) {
      mb1240_pulse_on_flag = 1;
    } else {
      mb1240_last_length = mb1240_length;
      mb1240_pulse_on_flag = 0;
      mb1240_length = 0;
      
      if (last_second != get_seconds()) {
        last_second = get_seconds();
        mb1240_count[last_second % MB1240_RATE_COUNT] = 0;
        u32 sum = 0;
        for (u8 i = 0; i < MB1240_RATE_COUNT; ++i) {
          if (i != last_second) {
            sum += mb1240_count[i];
          }
        }
        mb1240_rate = sum / (MB1240_RATE_COUNT - 1);
      }
      ++mb1240_count[get_seconds() % MB1240_RATE_COUNT];
    }
  }
  
}



MB1240_IRQHandler
{
  if (TIM_GetITStatus(MB1240_TIM, TIM_IT_Update) != RESET) {    
    TIM_ClearITPendingBit(MB1240_TIM, TIM_IT_Update);
    if (gpio_read_input(MB1240_GPIO)) {
      ++mb1240_length;
    }
  }  
}

u32 mb1240_get_length(void)
{
    return mb1240_length;
}

u32 mb1240_get_last_length(void)
{
  return mb1240_last_length;
}

u16 mb1240_get_rate(void)
{
  return mb1240_rate;
}



