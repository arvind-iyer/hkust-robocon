#include "ultrasonic.h"

static u32 count = 0, successful_count = 0;
static u32 pulse_width = 0;
static u32 pulse_width_history[ULTRASONIC_ECHO_PULSE_COUNT] = {0};
static u32 edge_trigger_us = 0, edge_falling_us = 0;
void ultrasonic_init(void)
{
  // GPIO init
  gpio_init(ULTRASONIC_TRIG_GPIO, GPIO_Speed_50MHz, GPIO_Mode_Out_PP, 1);
  gpio_init(ULTRASONIC_ECHO_GPIO, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING, 1);
  gpio_write(ULTRASONIC_TRIG_GPIO, (BitAction) 0);
  
 	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      									// TimeBase is for timer setting   > refer to P. 344 of library
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  EXTI_InitTypeDef   EXTI_InitStructure;
  
	RCC_APB2PeriphClockCmd(ULTRASONIC_RCC , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	ULTRASONIC_TIM->PSC = SystemCoreClock / 100000 - 1;		// Prescaler
//	ULTRASONIC_TIM->ARR = 1;
//	ULTRASONIC_TIM->EGR = 1;
//	ULTRASONIC_TIM->SR = 0;
//	ULTRASONIC_TIM->DIER = 1;
//	ULTRASONIC_TIM->CR1 = 1;
	
	TIM_TimeBaseStructure.TIM_Period = 60000;	                 				         // 60000 us
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1;     // 72M/1M - 1 = 71
	TIM_TimeBaseInit(ULTRASONIC_TIM, &TIM_TimeBaseStructure);      							 // this part feeds the parameter we set above


    
	TIM_ClearITPendingBit(ULTRASONIC_TIM, TIM_IT_Update);												 // Clear Interrupt bits
	TIM_ITConfig(ULTRASONIC_TIM, TIM_IT_Update, ENABLE);													 // Enable TIM Interrupt
  TIM_ITConfig(ULTRASONIC_TIM, TIM_IT_CC1, ENABLE);													 // Enable TIM Interrupt
	TIM_Cmd(ULTRASONIC_TIM, ENABLE);																							 // Counter Enable
  TIM_SetCounter(ULTRASONIC_TIM, 0);


  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //set "high" to be effective output
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	              //produce output when counter < CCR
  TIM_OCInitStructure.TIM_Pulse = ULTRASONIC_TRIG_PULSE - 1;                                     // 10us
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM_OC1Init(ULTRASONIC_TIM, &TIM_OCInitStructure);
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource1);
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
   
  TIM_ARRPreloadConfig(ULTRASONIC_TIM, ENABLE);
  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = ULTRASONIC_IRQn;
	NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
  
}

u32 get_pulse_width(void)
{
   return pulse_width; 
}

u32 get_distance(void)
{
  return pulse_width * 10 / 58;
}

u32 ultrasonic_get_distance_avg(void)
{
  u32 sum = 0;
  u8 successful_num = 0;
  for (u8 i = 0; i < ULTRASONIC_ECHO_PULSE_COUNT; ++i) {
    if (pulse_width_history[i] == 0) {
      // Error
      return 0;
    } else {
      successful_num++;
      sum += pulse_width_history[i];
    }
  }
  
  if (successful_num) {
    return sum * 10 / successful_num / 58;
  } else {
    return 0;
  }
  
  
}

u32 ultrasonic_get_count(void)
{
  return count;
}

u32 ultrasonic_get_successful_count(void)
{
  return successful_count;
}

/*
void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
    if (gpio_read_input(ULTRASONIC_ECHO_GPIO) == 1) {
      // Trigger
      edge_trigger_us = TIM_GetCounter(ULTRASONIC_TIM);
    } else {
      edge_falling_us = TIM_GetCounter(ULTRASONIC_TIM);
      if (edge_trigger_us == 0) {
        pulse_width = 0;
      } else if (edge_trigger_us > edge_falling_us) {
        pulse_width = 1;
      } else {
        pulse_width = edge_falling_us - edge_trigger_us;
        edge_trigger_us = edge_falling_us = 0;
        ++successful_count;
      } 
      pulse_width_history[count % ULTRASONIC_ECHO_PULSE_COUNT] = pulse_width; 
    }
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}
ULTRASONIC_IRQHandler
{
//    // 10 us pulse
//    if (count == 0) {
//      gpio_write(ULTRASONIC_TRIG_GPIO, 1);
//    } else if (count == 1) {
//      gpio_write(ULTRASONIC_TRIG_GPIO, 0);
//      pulse_width_count = 0;
//    }
//    
//    if (gpio_read_input(ULTRASONIC_ECHO_GPIO)) {
//      ++pulse_width_count;
//    } 
//    
//    ++count;
//    if (count == 6000) { // 60ms
//      count = 0;
//      if (pulse_width_count == 6000 - 1) {
//        pulse_width_count = 0; 
//      } else {
//        pulse_width = pulse_width_count;
//        pulse_width_count = 0;
//      }
//    }
    
  if (TIM_GetITStatus(ULTRASONIC_TIM, TIM_IT_Update) != RESET) {    
    gpio_write(ULTRASONIC_TRIG_GPIO, (BitAction) 1);
    edge_trigger_us = edge_falling_us = 0;
    ++count;
    TIM_ClearITPendingBit(ULTRASONIC_TIM, TIM_IT_Update);
  }
  
  
  if (TIM_GetITStatus(ULTRASONIC_TIM, TIM_IT_CC1) != RESET) { 
    gpio_write(ULTRASONIC_TRIG_GPIO, (BitAction) 0);
    TIM_ClearITPendingBit(ULTRASONIC_TIM, TIM_IT_CC1);
  }
}





*/
