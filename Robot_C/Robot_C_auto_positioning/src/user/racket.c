#include "racket.h"

static s32 racket_vel = 0;
static CLOSE_LOOP_FLAG loop_flag = OPEN_LOOP;
static s32 racket_cal_vel = -80;
static s32 racket_hit_vel =  80;
static s32 init_encoder_reading = -5000;
void racket_init(void)
{
	//Set up special character handlers
	special_char_handler_init();
	register_special_char_function(KEY_LOCK_RACKET, racket_lock);//o
	register_special_char_function(KEY_STOP_RACKET, racket_stop);//p
	register_special_char_function(KEY_HIT_RACKET, racket_hit);//k
	register_special_char_function(KEY_CALIB_RACKET, racket_calibrate);//l
	
	
	
	
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
	if((init_encoder_reading - get_encoder_value(RACKET))   > 6000)
	{
		racket_stop();
		init_encoder_reading = get_encoder_value(RACKET);
	}
	motor_set_vel(RACKET, racket_vel, loop_flag);
	
}


static void racket_stop(void)
{
	racket_set_vel(0, OPEN_LOOP);
}

static void racket_lock(void)
{
	racket_set_vel(0, CLOSE_LOOP);
}

static void racket_calibrate(void)
{
	racket_set_vel(racket_cal_vel, OPEN_LOOP);
}



static void racket_hit(void)
{
	if(button_pressed(RACKET_SWITCH))//pe2
	{
		racket_set_vel(racket_hit_vel, OPEN_LOOP);
		init_encoder_reading =  get_encoder_value(RACKET);
	}
	else
	{
		racket_calibrate();
	}	
}


static void racket_set_vel(s32 vel, CLOSE_LOOP_FLAG loop)
{
	racket_vel = vel;
	loop_flag = loop;
}	

static s32 racket_get_vel()
{
	return racket_vel;
}



RACKET_SWITCH_INTERRUPT_HANDLER
{
  if(EXTI_GetITStatus(RACKET_SWITCH_LINE) != RESET) 
	{
    racket_lock();
		racket_update();
    EXTI_ClearITPendingBit(RACKET_SWITCH_LINE);
  }
}

ROTATE_SWITCH_INTERRUPT_HANDLER
{
	if(EXTI_GetITStatus(ROTATE_SWITCH_LINE) != RESET)
	{
		racket_lock();
		racket_update();
		EXTI_ClearITPendingBit(ROTATE_SWITCH_LINE);
	}
}
