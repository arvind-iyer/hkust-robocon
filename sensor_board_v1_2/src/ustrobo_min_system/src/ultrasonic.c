#include "ultrasonic.h"
#include "ticks.h"
#include "can_protocol.h"
static u16 last_sample_second = (u16) -1;
static u8 last_sample_count = 0;
static u8 sample_count = 0;

static US_TypeDef us_devices[] = {
	{			// S0
		.trig_gpio = &PB10,
		.echo_gpio = &PA4
	}, {	// S1
		.trig_gpio = &PB10, 
		.echo_gpio = &PA5
	}, {	// S2
		.trig_gpio = &PB10,
		.echo_gpio = &PA6
	}, {	// S3
		.trig_gpio = &PC4,
		.echo_gpio = &PB1
	}, {	// S4
		.trig_gpio = &PC4,
		.echo_gpio = &PB0
	}, {	// S5
		.trig_gpio = &PC4,
		.echo_gpio = &PA7
	}, {	// S6
		.trig_gpio = &PC5,
		.echo_gpio = &PB11
	}, {	// S7
		.trig_gpio = &PC5,
		.echo_gpio = &PB12	
	}, {	// S8
		.trig_gpio = &PC5,
		.echo_gpio = &PB13
	}, {	// S9
		.trig_gpio = &PC11,
		.echo_gpio = &PC10	
	}, {	// S10
		.trig_gpio = &PC11,
		.echo_gpio = &PB15	
	}, {	// S11
		.trig_gpio = &PC11,
		.echo_gpio = &PB14
	}, {	// S12
		.trig_gpio = &PC12,
		.echo_gpio = &PB9	
	}, {	// S13
		.trig_gpio = &PC12,
		.echo_gpio = &PB8	
	}, {	// S14
		.trig_gpio = &PC12,
		.echo_gpio = &PD2	
	}
	
};


static u8 current_us = 0;		// For US_TAKE_TURN
//static u16 us_take_turn_break = 0;

void us_init(void)
{
	if (US_DEVICE_COUNT == 0) {return;}
	// GPIO init
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); 
	
	for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
		gpio_init(us_devices[i].trig_gpio, GPIO_Speed_50MHz, GPIO_Mode_Out_PP, 1);
		gpio_init(us_devices[i].echo_gpio, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING, 1);
		us_devices[i].trigger_time_us = 0;
		us_devices[i].falling_time_us = 0;
		us_devices[i].pulse_width_tmp = 0;
		us_devices[i].pulse_width = 0;
		us_devices[i].state = US_NULL;
		
		gpio_write(us_devices[i].trig_gpio, (BitAction) 0);
	}

	last_sample_count = 0;
	last_sample_second = (u16) -1; 
	sample_count = 0;

 	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      									// TimeBase is for timer setting   > refer to P. 344 of library
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  EXTI_InitTypeDef   EXTI_InitStructure;
  
	RCC_APB1PeriphClockCmd(US_RCC , ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1;     // 72M/1M - 1 = 71
	TIM_TimeBaseStructure.TIM_Period = US_RESET_TIME;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(US_TIM, &TIM_TimeBaseStructure);      							 // this part feeds the parameter we set above
	
	
    
	TIM_ClearITPendingBit(US_TIM, TIM_IT_Update);												 // Clear Interrupt bits
	TIM_ITConfig(US_TIM, TIM_IT_Update, ENABLE);													 // Enable TIM Interrupt
  TIM_ITConfig(US_TIM, TIM_IT_CC1, ENABLE);													 // Enable TIM Interrupt
	TIM_Cmd(US_TIM, ENABLE);																							 // Counter Enable
  TIM_SetCounter(US_TIM, 0);
	
	
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //set "high" to be effective output
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	              //produce output when counter < CCR
  TIM_OCInitStructure.TIM_Pulse = US_TRIG_PULSE;                                     // 10us
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM_OC1Init(US_TIM, &TIM_OCInitStructure);

  TIM_ARRPreloadConfig(US_TIM, ENABLE);
	
	
	
	// EXIT & NVIC Init
	
	
	
	// S0: PA4
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	// S1: PA5
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);
  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;  
	NVIC_Init(&NVIC_InitStructure);	
	
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = US_IRQn;
	NVIC_Init(&NVIC_InitStructure);
  



}

static void us_trigger(u8 i)
{
	us_devices[i].trigger_time_us = 0;
	us_devices[i].falling_time_us = 0;
	gpio_write(us_devices[i].trig_gpio, (BitAction) 1); // Trigger	
}


u32 us_get_pulse_raw(u8 i)
{
	if (i >= US_DEVICE_COUNT) {return 0;}
  return us_devices[i].pulse_width_tmp;
}

u32 us_get_pulse(u8 i)
{
	if (i >= US_DEVICE_COUNT) {return 0;}
   return us_devices[i].pulse_width; 
}

u32 us_get_distance(u8 i)
{
	if (i >= US_DEVICE_COUNT) {return 0;}
  return us_devices[i].pulse_width * 34 / 100 / 2;
}

u8 us_get_speed(void)
{
	if (last_sample_second != get_seconds()) {
		return 0;
	}
	return last_sample_count; 
}

u8 us_get_current_us(void)
{
	return current_us;
}

void us_can_tx(u8 id, u16 distance)
{
	CAN_MESSAGE msg;
	msg.id = (US_CAN_ID + id);
	msg.length = 3;
	msg.data[0] = US_CAN_DISTANCE_CMD;
	msg.data[1] = (u8) ((distance >> 8) & 0xFF);
	msg.data[2] = (u8) (distance & 0xFF);
	
	can_tx_enqueue(msg);
}

void us_echo_interrupt(u8 i)
{
	u16 counter = TIM_GetCounter(US_TIM); 
	u8 signal = gpio_read_input(us_devices[i].echo_gpio);
	if (us_devices[i].echo_gpio_state == signal) {
		// Not toggling (not fulfulling EXTI)
		return;
	}
	us_devices[i].echo_gpio_state = signal;
	
	if (signal && us_devices[i].trigger_time_us == 0) {
		us_devices[i].trigger_time_us = counter;
	} else if (!signal && us_devices[i].trigger_time_us > 0) {
		us_devices[i].falling_time_us = counter;
		if (us_devices[i].trigger_time_us == 0) {
			us_devices[i].pulse_width = 0;
		} else if (us_devices[i].trigger_time_us < us_devices[i].falling_time_us) {
			us_devices[i].pulse_width = us_devices[i].falling_time_us - us_devices[i].trigger_time_us;
		}
		us_can_tx(i, us_get_distance(i));
	}
}



// S3: PC9, S4: PC7, S8: PB5, S9: PB7
void EXTI9_5_IRQHandler(void) 
{
  if(EXTI_GetITStatus(EXTI_Line5) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line5);
		us_echo_interrupt(1);
  }
	
}

// S0: PA4
void EXTI4_IRQHandler(void) 
{
  if(EXTI_GetITStatus(EXTI_Line4) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line4);
		us_echo_interrupt(0); 
  }
}



US_IRQHandler
{
  if (TIM_GetITStatus(US_TIM, TIM_IT_Update) != RESET) {   
		// RESET
		TIM_ClearITPendingBit(US_TIM, TIM_IT_Update);
		for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
			us_can_tx(i, us_get_distance(i));
			us_trigger(i);
			// Reset
			us_devices[i].echo_gpio_state = gpio_read_input(us_devices[i].echo_gpio);
			us_devices[i].pulse_width = 0;
		}
    if (last_sample_second != get_seconds()) {
				last_sample_second = get_seconds(); 
				last_sample_count = sample_count; 
				sample_count = 0;
		}
		++sample_count; 
  }
  
  
  if (TIM_GetITStatus(US_TIM, TIM_IT_CC1) != RESET) { 
		TIM_ClearITPendingBit(US_TIM, TIM_IT_CC1);
		for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
			us_devices[i].trigger_time_us = 0;
			us_devices[i].falling_time_us = 0;
			gpio_write(us_devices[i].trig_gpio, (BitAction) 0); // Trigger
		}
    
  } 
}





