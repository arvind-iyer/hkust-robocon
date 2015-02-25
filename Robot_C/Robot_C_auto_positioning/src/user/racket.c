#include "racket.h"

static s32 racket_vel = 0;
static CLOSE_LOOP_FLAG loop_flag = OPEN_LOOP;
static s32 racket_cal_vel = 500;
static s32 racket_hit_vel = -1300;
static u32 racket_serve_delay = 510;
static s32 init_encoder_reading = -5000;
static u8 allow_hit = 0;
static u8 is_locked = 0;
static u8 is_servo_release = 0;
static u8 serve_enabled=0;
static u32 racket_serve_start_time=0;
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
//	EXTI_InitTypeDef   EXTI_InitStructure;
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, RACKET_PIN_SOURCE);
//  EXTI_InitStructure.EXTI_Line = RACKET_SWITCH_LINE;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);
//	
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStructure.NVIC_IRQChannel = RACKET_IRQn;
//	NVIC_Init(&NVIC_InitStructure);

//	
//	//For ROTATE_SWITCH at PE3
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, ROTATE_PIN_SOURCE);
//  EXTI_InitStructure.EXTI_Line = ROTATE_SWITCH_LINE;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStructure.NVIC_IRQChannel = ROTATE_IRQn;
//	NVIC_Init(&NVIC_InitStructure);


	
}

void racket_serve_increase_delay(void)
{
	racket_serve_delay+=5;
}

void racket_serve_decrease_delay(void)
{
	racket_serve_delay-=5;
}

void racket_increase_hit_vel(void)
{
	racket_hit_vel-=2;
}

void racket_decrease_hit_vel(void)
{
	racket_hit_vel+=2;
}


void racket_update(void)
{
	if (serve_enabled && get_full_ticks()>racket_serve_delay+racket_serve_start_time)		// execute hit_racket() after serve delay
	{
		serve_enabled=0;
		racket_hit();
		toggle_servo();
	}
	
	
	if(get_encoder_value(RACKET) > (2000+racket_hit_vel) && allow_hit)		// during racket_hit(), lock the motor if the racket pass the encoder point.
	{
		motor_set_acceleration(RACKET, 1000);
		racket_lock();
		
		racket_hit_off();
		
	}
	else if(button_pressed(RACKET_SWITCH) || button_pressed(ROTATE_SWITCH))		// if any of the mechanical switch is pressed, lock the motor.
	{
		if(!allow_hit)
		{
			is_locked = 1;
			racket_lock();
	
		}
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
	racket_set_vel(racket_cal_vel, OPEN_LOOP);
}


void racket_hit_on(void)
{
	allow_hit = 1;
}

void racket_hit_off(void)
{
	allow_hit = 0;
}

void toggle_servo(void)
{
	servo_control(SERVO4, (is_servo_release ? 900 : 1500));
	is_servo_release = !is_servo_release;
	
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

void racket_start_serve(void)
{
	if (!is_servo_release)
	{
		toggle_servo();
		racket_serve_start_time = get_full_ticks();
		serve_enabled=1;
	}
	
}


void racket_set_vel(s32 vel, CLOSE_LOOP_FLAG loop)
{
	racket_vel = vel;
	loop_flag = loop;
}	

s32 racket_get_vel()
{
	return racket_hit_vel;
}

u32 racket_get_serve_delay()
{
	return racket_serve_delay;
}

u8 get_lock_status(void)
{
	return is_locked;
}

//RACKET_SWITCH_INTERRUPT_HANDLER
//{
//	
//  if(EXTI_GetITStatus(RACKET_SWITCH_LINE) != RESET) 
//	{
//		EXTI_ClearFlag(RACKET_SWITCH_LINE);
//		EXTI_ClearITPendingBit(RACKET_SWITCH_LINE);
//		if(allow_hit)
//			return;
//		is_locked = 1;
//    racket_lock();
//		racket_update();
//   }
//	 
//  
//}

//ROTATE_SWITCH_INTERRUPT_HANDLER
//{
//	
//	
//	if(EXTI_GetITStatus(ROTATE_SWITCH_LINE) != RESET)
//	{
//		EXTI_ClearITPendingBit(ROTATE_SWITCH_LINE);
//		EXTI_ClearFlag(ROTATE_SWITCH_LINE);
//		if(allow_hit)
//			return;
//		is_locked = 1;
//		racket_lock();
//		racket_update();
//	}
//		
//}
