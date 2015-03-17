#include "encoder.h"

static Encoder_Typedef encoder = {{ENCODER_TIMER1,ENCODER_TIMER1_CLOCK_SOURCE,ENCODER_TIMER1_GPIO_CLOCK_SOURCE,
																	 ENCODER_TIMER1_PORT1,ENCODER_TIMER1_PORT2,ENCODER_TIMER1_GPIOx},
																	{ENCODER_TIMER2,ENCODER_TIMER2_CLOCK_SOURCE,ENCODER_TIMER2_GPIO_CLOCK_SOURCE,
																	 ENCODER_TIMER2_PORT1,ENCODER_TIMER2_PORT2,ENCODER_TIMER2_GPIOx}};

/**
  * @brief  Initialization of encoder
  * @param  None
  * @retval None
  */
void encoder_init(void){
	u8 encoder_id;

//	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	for (encoder_id = 0; encoder_id < ENCODER_NO; ++encoder_id) {
		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		// Clock source enable
		RCC_APB1PeriphClockCmd(encoder[encoder_id].clock_source, ENABLE);																// Enable Timer Clock Source
		RCC_APB2PeriphClockCmd(encoder[encoder_id].GPIO_clock_source | RCC_APB2Periph_AFIO, ENABLE);		// Enable GPIO, clock
		
		// GPIO init
		GPIO_StructInit(&GPIO_InitStructure);																														// Set to default
		GPIO_InitStructure.GPIO_Pin = encoder[encoder_id].port1 | encoder[encoder_id].port2;						// Set Timer Pin 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;																						// Set Pin mode to floating
		GPIO_Init(encoder[encoder_id].GPIOx, &GPIO_InitStructure);																			// Activate
		if (encoder_id == 0) {
			GPIO_PinRemapConfig(GPIO_Remap_TIM4 , ENABLE);
		}
		
		// Timer init
		TIM_DeInit(encoder[encoder_id].timer);																													// clear
		TIM_TimeBaseStructure.TIM_Prescaler = 0x00; 																										// No prescaling
		TIM_TimeBaseStructure.TIM_Period = 0xffff;																											// Max Count
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(encoder[encoder_id].timer, &TIM_TimeBaseStructure);
		
		// Setting to Rising edge mode
		TIM_EncoderInterfaceConfig(encoder[encoder_id].timer, TIM_EncoderMode_TI12,
														 TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
		
		TIM_SetCounter(encoder[encoder_id].timer, 0);																													// Reset Count as 0
		TIM_Cmd(encoder[encoder_id].timer, ENABLE);																														// Enable counter timer

	}
}

/**
  * @brief  Get the count reading from encoder.
	* @param  ENCODERx: where x can be 1 to 2
  * @retval The reading of the encoder
  */
u32 get_count(ENCODER ENCODERx){
	return TIM_GetCounter(encoder[ENCODERx].timer);
}
