#include "racket.h"

static s32 racket_vel = 0;
static CLOSE_LOOP_FLAG loop_flag = OPEN_LOOP;
static s32 racket_cal_vel = 8;
static s32 racket_hit_vel = -150;
static s32 init_encoder_reading = -5000;
static u8 allow_hit = 0;
static u8 is_locked = 0;
void racket_init(void)
{
	//Set up special character handlers
//	special_char_handler_init();
//	register_special_char_function(KEY_LOCK_RACKET, racket_lock);//o
//	register_special_char_function(KEY_STOP_RACKET, racket_stop);//p
//	register_special_char_function(KEY_HIT_RACKET, racket_hit);//k
//	register_special_char_function(KEY_CALIB_RACKET, racket_calibrate);//l
	
	
	
	
	// Initialise interrupt - NVIC and EXT(2 and 3) channels
	
	//For Racket_Switch at PE2
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, RACKET_PIN_SOURCE);
  EXTI_InitStructure.EXTI_Line = RACKET_SWITCH_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
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
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = ROTATE_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
}



void racket_update(void)
{
	
	if(get_encoder_value(RACKET) > 2000 && allow_hit)
	{
		racket_lock();
		racket_hit_off();
	}
	motor_set_vel(RACKET, racket_vel, loop_flag);
	
}

s32 get_init_enc(void)
{
	return init_encoder_reading;
}
void racket_stop(void)
{
	racket_hit_off();
	is_locked = 0;
	racket_set_vel(0, OPEN_LOOP);
}

void racket_lock(void)
{
	racket_hit_off();
	racket_set_vel(0, CLOSE_LOOP);
}

void racket_calibrate(void)
{
	racket_hit_off();
	racket_set_vel(racket_cal_vel, CLOSE_LOOP);
}


void racket_hit_on(void)
{
	allow_hit = 1;
}

void racket_hit_off(void)
{
	allow_hit = 0;
}

void racket_hit(void)
{
	
	if(is_locked)//pe3
	{
		init_encoder_reading =  get_encoder_value(RACKET);
		is_locked = 0;
		racket_hit_on();
		racket_vel = racket_hit_vel;
		loop_flag = OPEN_LOOP;
	}
	else
	{
		racket_calibrate();
		
	}	
	
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

u8 get_lock_status(void)
{
	return is_locked;
}

RACKET_SWITCH_INTERRUPT_HANDLER
{
	
  if(EXTI_GetITStatus(RACKET_SWITCH_LINE) != RESET) 
	{
		EXTI_ClearFlag(RACKET_SWITCH_LINE);
		EXTI_ClearITPendingBit(RACKET_SWITCH_LINE);
		if(allow_hit)
			return;
		is_locked = 1;
    racket_lock();
		racket_update();
   }
	 
  
}

ROTATE_SWITCH_INTERRUPT_HANDLER
{
	
	
	if(EXTI_GetITStatus(ROTATE_SWITCH_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(ROTATE_SWITCH_LINE);
		EXTI_ClearFlag(ROTATE_SWITCH_LINE);
		if(allow_hit)
			return;
		is_locked = 1;
		racket_lock();
		racket_update();
	}
		
}
