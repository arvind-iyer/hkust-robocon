#include "nec_protocol.h"
#include "stm32f10x.h"                  // Device header

#include "stm32f10x_tim.h"

NEC_PORT_TypeDef nec_ports[] = {
	{
		.gpio = &PA0,
		.TIMx = TIM2,
		.TIM_Channelx = TIM_Channel_1,
		.TIM_CCx = TIM_IT_CC1
	},
	{
		.gpio = &PA1,
		.TIMx = TIM2,
		.TIM_Channelx = TIM_Channel_2,
		.TIM_CCx = TIM_IT_CC2
	},
	{
		.gpio = &PA2,
		.TIMx = TIM2,
		.TIM_Channelx = TIM_Channel_3,
		.TIM_CCx = TIM_IT_CC3
	},
	{
		.gpio = &PA3,
		.TIMx = TIM2,
		.TIM_Channelx = TIM_Channel_4,
		.TIM_CCx = TIM_IT_CC4
	},	
	
	{
		.gpio = &PC9,
		.TIMx = TIM3,
		.TIM_Channelx = TIM_Channel_4,
		.TIM_CCx = TIM_IT_CC4
	},
	{
		.gpio = &PC8,
		.TIMx = TIM3,
		.TIM_Channelx = TIM_Channel_3,
		.TIM_CCx = TIM_IT_CC3
	},
	{
		.gpio = &PC7,
		.TIMx = TIM3,
		.TIM_Channelx = TIM_Channel_3,
		.TIM_CCx = TIM_IT_CC3
	},
	{
		.gpio = &PC6,
		.TIMx = TIM3,
		.TIM_Channelx = TIM_Channel_3,
		.TIM_CCx = TIM_IT_CC3
	},
};

void NEC_init(void)
{
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	
	for (u8 i = 0; i < sizeof(nec_ports) / sizeof(NEC_PORT_TypeDef); ++i) {
		// Input capture (IC init)
		NEC_PORT_TypeDef* nec_port = &nec_ports[i];
		nec_port->TIM_ICInitStructure.TIM_Channel = nec_port->TIM_Channelx;
		nec_port->TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
		nec_port->TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		nec_port->TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		nec_port->TIM_ICInitStructure.TIM_ICFilter = 0x0;
		
		TIM_ICInit(nec_port->TIMx, &nec_port->TIM_ICInitStructure);
	  /* TIM enable counter */
		TIM_Cmd(nec_port->TIMx, ENABLE);
	  /* Enable the CCx Interrupt Request */
		TIM_ITConfig(nec_port->TIMx, nec_port->TIM_CCx, ENABLE);
	
	}
	
	// NVIC config
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

