#include "encoder.h"

#define ENCODER_TIM_PERIOD (u16)0xffff   // number of pulses per revolution
#define COUNTER_RESET   (u16)0
#define MAX_COUNT       (u16)20000		 // cannot count to this value

s32 hw_encoder_Count[3];

TIM_TypeDef* ENCODER[3] = {TIM2 , TIM3 , TIM4};

void hw_encoder_Init(void)
{ 
	/* Encoder unit connected to TIM3, 4X mode */    
	GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM3 clock source enable */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM2 , ENABLE);
	/* Enable GPIOC, clock */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_6;	// TIM4	 encoder2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	/* Configure PB.06,07 as encoder2 input */
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	// TIM3  encoder1
	/* Configure PC.06,07 as encoder1 	 */
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); // remap TIM3 PORT

	/* Configure PA.00,01 as encoder0 input */		// TIM2	encoder0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Timer configuration in Encoder mode */
	encoder_mode(TIM2);
	encoder_mode(TIM3);
	encoder_mode(TIM4);
}

/* Timer configuration in Encoder mode */ 
void encoder_mode(TIM_TypeDef* ENCODER_TIMER)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_DeInit(ENCODER_TIMER);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(ENCODER_TIMER, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(ENCODER_TIMER, TIM_EncoderMode_TI12, 
							 TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	//Reset counter
	ENCODER_TIMER->CNT = COUNTER_RESET; 
	TIM_Cmd(ENCODER_TIMER, ENABLE);
}

/*This function for the privent 16 bit overflow */
s16 hw_encoder_cal_count(u8 motor)
{
	static u16   lastCount[3] = {0,0,0};
	u16 curCount = ENCODER[motor]->CNT;      // TIMx counter is 0 to 65535
	s32 dAngle = curCount - lastCount[motor];

	if (dAngle >= MAX_COUNT)					 // routa anti-clockwise
	{
		dAngle -= ENCODER_TIM_PERIOD;			 
	}
	else if (dAngle < -MAX_COUNT)			 // routa clockwise
	{
		dAngle += ENCODER_TIM_PERIOD;
	}
	lastCount[motor] = curCount;
	return (s16)dAngle;
}

