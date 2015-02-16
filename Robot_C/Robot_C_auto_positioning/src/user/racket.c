#include "racket.h"

static s32 racket_vel = 0;
static CLOSE_LOOP_FLAG loop_flag = OPEN_LOOP;


void racket_init(void)
{
	// Initialise interrupt - NVIC and EXT(2 and 3) channels
	
	//For Racket_Switch at PE2
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, RACKET_PIN_SOURCE);
  EXTI_InitStructure.EXTI_Line = RACKET_SWITCH_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = RACKET_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	
	//For ROTATE_SWITCH at PE3
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, ROTATE_PIN_SOURCE);
  EXTI_InitStructure.EXTI_Line = ROTATE_SWITCH_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = ROTATE_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	
	//Set up special character handlers
	register_special_char_function(KEY_LOCK_RACKET, racket_lock);
	register_special_char_function(KEY_STOP_RACKET, racket_stop);
}

void racket_update(void)
{
	motor_set_vel(RACKET, racket_vel, loop_flag);
}


void racket_stop(void)
{
	racket_vel = 0;
	loop_flag = OPEN_LOOP;
}

void racket_lock(void)
{
	racket_vel = 0;
	loop_flag = CLOSE_LOOP;
}

void racket_set_vel(s32 vel, CLOSE_LOOP_FLAG loop)
{
	racket_vel = vel;
	loop_flag = loop;
}	

s32 racket_get_vel()
{
	return racket_vel;
}

RACKET_SWITCH_INTERRUPT_HANDLER
{
  if(EXTI_GetITStatus(RACKET_SWITCH_LINE) != RESET) 
	{
    racket_lock();
    EXTI_ClearITPendingBit(RACKET_SWITCH_LINE);
  }
}

ROTATE_SWITCH_INTERRUPT_HANDLER
{
	if(EXTI_GetITStatus(ROTATE_SWITCH_LINE) != RESET)
	{
		racket_lock();
		EXTI_ClearITPendingBit(ROTATE_SWITCH_LINE);
	}
}
