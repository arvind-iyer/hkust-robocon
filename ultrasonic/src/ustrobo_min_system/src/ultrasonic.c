#include "ultrasonic.h"
#include "ticks.h"
#include "can_protocol.h"

static US_TypeDef us_devices[] = {
	{
		.trig_gpio = &PE0,
		.echo_gpio = &PE1
	}, {
		.trig_gpio = &PE2, 
		.echo_gpio = &PE3
	}, {
		.trig_gpio = &PE4,
		.echo_gpio = &PE5
	}, {
		.trig_gpio = &PE6,
		.echo_gpio = &PE7
	}, {
		.trig_gpio = &PE8,
		.echo_gpio = &PE9
	}, {
		.trig_gpio = &PE10,
		.echo_gpio = &PE11
	}, {
		.trig_gpio = &PE12,
		.echo_gpio = &PE13
	}, {
		.trig_gpio = &PE14,
		.echo_gpio = &PE15
	}
	
};


static US_MODE us_mode = US_INDEPENDENT;

static u8 current_us = 0;		// For US_TAKE_TURN
static u16 us_take_turn_break = 0;
void us_init(US_MODE mode)
{
	if (US_DEVICE_COUNT == 0) {return;}
  // GPIO init
	for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
		gpio_init(us_devices[i].trig_gpio, GPIO_Speed_50MHz, GPIO_Mode_Out_PP, 1);
		gpio_init(us_devices[i].echo_gpio, GPIO_Speed_50MHz, GPIO_Mode_IPD, 1);
		us_devices[i].pulse_width_tmp = 0;
		us_devices[i].pulse_width = 0;
		us_devices[i].idle_width = 0;
		us_devices[i].state = US_NULL;
		gpio_write(us_devices[i].trig_gpio, (BitAction) 0);
	}

 	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      									// TimeBase is for timer setting   > refer to P. 344 of library
  //TIM_OCInitTypeDef  TIM_OCInitStructure;
 // EXTI_InitTypeDef   EXTI_InitStructure;
  
	RCC_APB2PeriphClockCmd(US_RCC , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	US_TIM->PSC = SystemCoreClock / 100000 - 1;		// Prescaler
//	US_TIM->ARR = 1;
//	US_TIM->EGR = 1;
//	US_TIM->SR = 0;
//	US_TIM->DIER = 1;
//	US_TIM->CR1 = 1;
	
	TIM_TimeBaseStructure.TIM_Period = 1;	                 				         // 60000 us
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 100000 - 1;     // 72M/1M - 1 = 71
	TIM_TimeBaseInit(US_TIM, &TIM_TimeBaseStructure);      							 // this part feeds the parameter we set above


    
	TIM_ClearITPendingBit(US_TIM, TIM_IT_Update);												 // Clear Interrupt bits
	TIM_ITConfig(US_TIM, TIM_IT_Update, ENABLE);													 // Enable TIM Interrupt
//  TIM_ITConfig(US_TIM, TIM_IT_CC1, ENABLE);													 // Enable TIM Interrupt
	TIM_Cmd(US_TIM, ENABLE);																							 // Counter Enable
  TIM_SetCounter(US_TIM, 0);
  
   
  TIM_ARRPreloadConfig(US_TIM, ENABLE);
  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = US_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
  current_us = 0;
	us_mode = mode;

}

US_MODE us_get_mode(void)
{
	return us_mode;
}

US_STATE us_get_state(u8 i)
{
	if (i >= US_DEVICE_COUNT) {return US_READY;}
	return us_devices[i].state;
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
  return us_devices[i].pulse_width * 34 / 10;
}

u16 us_get_speed(u8 i)
{
	if (i >= US_DEVICE_COUNT) {return 0;}
	return us_devices[i].last_sample_count;
}

u8 us_get_current_us(void)
{
	return current_us;
}

void us_can_tx(u8 id, u16 distance) {
	CAN_MESSAGE msg;
	msg.id = (US_CAN_ID + id);
	msg.length = 2;
	msg.data[0] = (u8) ((distance >> 8) & 0xFF);
	msg.data[1] = (u8) (distance & 0xFF);
	
	can_tx_enqueue(msg);
}


US_IRQHandler
{
  if (TIM_GetITStatus(US_TIM, TIM_IT_Update) != RESET) {  
		TIM_ClearITPendingBit(US_TIM, TIM_IT_Update);	 
		u8 rx_completed_count = 0;
		for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
			if (us_mode == US_TAKE_TURN) {
				if (i != current_us) {continue;}
			}
			
			US_TypeDef* us_device = &us_devices[i];
			switch (us_device->state) {
				case US_NULL:
					if (us_mode == US_TAKE_TURN) {
						if (i == current_us) {
							us_device->state = US_READY;
						} else {
							continue;
						}
					} else {
						us_device->state = US_READY;
					} 
				
				break;
				
				case US_READY: 
					// Start trigger
					
					gpio_write(us_device->trig_gpio, (BitAction) 1);
					us_device->state = US_TRIGGER;
					us_device->rx_complete = 0;
				break;
				
				case US_TRIGGER:
					// Triggered
					gpio_write(us_device->trig_gpio, (BitAction) 0);
					us_device->pulse_width_tmp = 0;
					us_device->idle_width = 0;
					us_device->state = US_WAITING_FOR_ECHO;
				break;
				
				case US_WAITING_FOR_ECHO:
					if (gpio_read_input(us_device->echo_gpio) == 1) {
						us_device->pulse_width_tmp = 0;
						us_device->state = US_ECHO_RAISED;
					} 
					
					if (us_device->idle_width++ >= US_IDLE_RESET_COUNT) {
						// Reset state if nothing received (no trigger for a long time)
						us_device->idle_width = 0;
						us_device->pulse_width = 0;
						us_device->pulse_width_tmp = 0;
						us_can_tx(i, us_get_distance(i));
						us_device->state = US_ECHO_FALLEN;
					}

				break;
				
				case US_ECHO_RAISED:
					++us_device->pulse_width_tmp;
					// Wait for edge falling
					if (gpio_read_input(us_device->echo_gpio) == 0 || us_device->pulse_width_tmp >= US_MAX_WIDTH) {
						us_device->pulse_width = us_device->pulse_width_tmp;
						us_device->pulse_width_tmp = 0;
						// CAN TX PULSE WIDTH
						us_can_tx(i, us_get_distance(i));
						
						if (us_device->last_sample_second != get_seconds()) {
							us_device->last_sample_second = get_seconds();
							us_device->last_sample_count = us_device->sample_count;
							us_device->sample_count = 0;			// Reset sample count
						}
						++us_device->sample_count;
						us_device->state = US_ECHO_FALLEN;
						us_take_turn_break = 0;
					}
				break;
				
				case US_ECHO_FALLEN:
					if (us_mode == US_INDEPENDENT) {
						us_device->state = US_READY;
					} else if (us_mode == US_SYNC) {
						++rx_completed_count;
					} else if (us_mode == US_TAKE_TURN) {
						++us_take_turn_break;
						if (us_take_turn_break >= US_TAKE_TURN_BREAK) {
							us_take_turn_break = 0;
							us_device->state = US_NULL;
							current_us = (current_us + 1) % US_DEVICE_COUNT; 
							us_devices[current_us].state = US_NULL;	// Next sensor
						}
					}
				break;
			}
		}
		
		if (us_mode == US_SYNC) {
			if (rx_completed_count >= US_DEVICE_COUNT) {
				for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
					us_devices[i].state = US_READY;
				}
			}
		}
  }
}



/*
void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
    if (gpio_read_input(US_ECHO_GPIO) == 1) {
      // Trigger
      edge_trigger_us = TIM_GetCounter(US_TIM);
    } else {
      edge_falling_us = TIM_GetCounter(US_TIM);
      if (edge_trigger_us == 0) {
        pulse_width = 0;
      } else if (edge_trigger_us > edge_falling_us) {
        pulse_width = 1;
      } else {
        pulse_width = edge_falling_us - edge_trigger_us;
        edge_trigger_us = edge_falling_us = 0;
        ++successful_count;
      } 
      pulse_width_history[count % US_ECHO_PULSE_COUNT] = pulse_width; 
    }
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}
US_IRQHandler
{
//    // 10 us pulse
//    if (count == 0) {
//      gpio_write(US_TRIG_GPIO, 1);
//    } else if (count == 1) {
//      gpio_write(US_TRIG_GPIO, 0);
//      pulse_width_count = 0;
//    }
//    
//    if (gpio_read_input(US_ECHO_GPIO)) {
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
    
  if (TIM_GetITStatus(US_TIM, TIM_IT_Update) != RESET) {    
    gpio_write(US_TRIG_GPIO, (BitAction) 1);
    edge_trigger_us = edge_falling_us = 0;
    ++count;
    TIM_ClearITPendingBit(US_TIM, TIM_IT_Update);
  }
  
  
  if (TIM_GetITStatus(US_TIM, TIM_IT_CC1) != RESET) { 
    gpio_write(US_TRIG_GPIO, (BitAction) 0);
    TIM_ClearITPendingBit(US_TIM, TIM_IT_CC1);
  }
}





*/
