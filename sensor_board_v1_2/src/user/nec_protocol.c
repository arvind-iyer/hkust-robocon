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
		.TIM_Channelx = TIM_Channel_2,
		.TIM_CCx = TIM_IT_CC2
	},
	{
		.gpio = &PC6,
		.TIMx = TIM3,
		.TIM_Channelx = TIM_Channel_1,
		.TIM_CCx = TIM_IT_CC1
	},
};

void NEC_init(void)
{
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
	// Timer base
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 100000 - 1;     // 72M/1M - 1 = 71
	TIM_TimeBaseStructure.TIM_Period = NEC_TIMER_PERIOD;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
	
	for (u8 i = 0; i < sizeof(nec_ports) / sizeof(NEC_PORT_TypeDef); ++i) {

		
		// Input capture (IC init)
		NEC_PORT_TypeDef* nec_port = &nec_ports[i];
		
		// GPIO init
		gpio_init(nec_port->gpio, GPIO_Speed_10MHz, GPIO_Mode_IPD, 1);
		
		nec_port->last_gpio_state = gpio_read_input(nec_port->gpio);
		
		nec_port->TIM_ICInitStructure.TIM_Channel = nec_port->TIM_Channelx;
		nec_port->TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		nec_port->TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		nec_port->TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		nec_port->TIM_ICInitStructure.TIM_ICFilter = 0x0;
		
		TIM_ICInit(nec_port->TIMx, &nec_port->TIM_ICInitStructure);
	  /* TIM enable counter */
		TIM_Cmd(nec_port->TIMx, ENABLE);
	  /* Enable the CCx Interrupt Request */
		TIM_ITConfig(nec_port->TIMx, nec_port->TIM_CCx, ENABLE);
	
		nec_port->deque_head = 0;
		nec_port->deque_tail = 0;
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

static void nec_handler(u8 id, u16 pulse)
{
	NEC_PORT_TypeDef* nec_port = &nec_ports[id];
	
	if (nec_port->last_gpio_state == gpio_read_input(nec_port->gpio)) {
		return;
	}
	nec_port->last_gpio_state = gpio_read_input(nec_port->gpio);
	
	
	nec_port->deque[nec_port->deque_tail] = pulse;
	
	nec_port->deque_tail = (nec_port->deque_tail + 1) % NEC_QUEUE_SIZE;
	
	
	CAN_MESSAGE msg;
	msg.length = 2;
	msg.id = 0x300 + id;
	msg.data[0] = (pulse >> 8) & 0xFF;
	msg.data[1] = pulse & 0xFF;
	can_tx_enqueue(msg); 
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) { 
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		nec_handler(0, TIM_GetCapture1(TIM2));
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET) { 
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET) { 
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET) { 
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
	}
	
}

//void TIM3_IRQHandler(void)
//{
//	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) { 
//		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
//	}
//	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET) { 
//		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
//	}
//	if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET) { 
//		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
//	}
//	if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET) { 
//		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
//	}
//}

