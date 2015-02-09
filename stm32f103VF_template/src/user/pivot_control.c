#include "pivot_control.h"

static bool interrupt_triggered = false;
static u32 interrupt_triggered_time;
static bool motor_on=false;
static bool switch_left_on = false;
static bool switch_right_on = false;
static bool left_mode_on = false;
static bool right_mode_on = false;
static u32 motor_triggered_time;

static u16 pivot_speed = 1800;

void pivot_init(void)         //need to change pin 
{
		register_special_char_function('u', pivot_turn_left);
		register_special_char_function('o', pivot_turn_right);

	/* GPIO configuration */
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);   
	
	// switch init
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = SWITCH_LEFT_Pin | SWITCH_RIGHT_Pin;
  GPIO_Init(SWITCH_GPIO, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource7);
	
	/* EXTI configuration */
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/* NVIC configuration */
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}

// error in this handler  change EXTI
void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
		interrupt_triggered_time = get_full_ticks();
		interrupt_triggered = true;
	  EXTI_ClearITPendingBit(EXTI_Line7);
	}
}


// bug not fixed yet, temporary version
void pivot_update(void){
	
	// switch check
	if (interrupt_triggered && get_full_ticks() - interrupt_triggered_time > 20) {
		if (GPIO_ReadInputDataBit(SWITCH_GPIO, SWITCH_LEFT_Pin)) {
			switch_left_on = true;
		} else {
			switch_left_on = false;
		}
		
		if (GPIO_ReadInputDataBit(SWITCH_GPIO, SWITCH_RIGHT_Pin)) {
			switch_right_on = true;
		} else {
			switch_right_on = false;
		}
		interrupt_triggered = false;
	}
	
	//left turning mode, set motor on if triggered, stop after 20ms
	if(left_mode_on){
		if(!switch_left_on){
			motor_triggered_time = get_full_ticks();
			motor_set_vel(MOTOR5, 30, OPEN_LOOP);     //direction unknown
			motor_on= true;
			
		}else{
			motor_lock(MOTOR7) ;
			motor_on = false;
		}	
	}
	
	//right turning mode, set motor on if triggered, stop after 20ms
	if(right_mode_on){
		if(!switch_right_on){
			motor_triggered_time = get_full_ticks();
			motor_set_vel(MOTOR5, -30, OPEN_LOOP);			//direction unknown
			motor_on= true;
		}else{
			motor_lock(MOTOR7) ;
			motor_on = false;
		}	
	}
	
	//stop motor after delay time
	if(motor_on && get_full_ticks() - motor_triggered_time > Motor_delay){
		motor_lock (MOTOR7);
		motor_on = false;
	}
	
}


void pivot_turn_left(void)
{
	
	left_mode_on= true;
	
}

void pivot_turn_right(void)
{
	right_mode_on = true;
	
}

u8 get_left_mode(void){
	return left_mode_on;
}
u8 get_right_mode(void){
	return right_mode_on;
}

u8 get_motor_mode (void){
	return motor_on;
}


