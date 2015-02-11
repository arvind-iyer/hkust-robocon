#include "lower_racket_control.h"
#include "delay.h"

static bool switch_on = false;
static bool calibrate_mode_on = false;
static bool calibrated = false;
static bool interrupt_triggered = false;
static u32 interrupt_triggered_time;
static u32 prev_encoder_value = 0;
static u32 current_encoder_distance = 0;

static u8 racket_mode = 0;
static u16 racket_speed = 1800;
static u16 racket_delay = 10;

u16 get_racket_speed() {
	return racket_speed;
}

u16 get_racket_delay() {
	return racket_delay;
}


void increase_racket_speed() {
	if(racket_speed < 1800)
		racket_speed += 5;
}

void decrease_racket_speed() {
	if(racket_speed > 0)
			racket_speed -= 5;
}

void increase_racket_delay() {
	racket_delay += 5;
}

void decrease_racket_delay() {
	racket_delay -= 5;
}


void racket_init(void)
{
	register_special_char_function('i', racket_received_command);
	register_special_char_function(']', racket_calibrate);
	
	register_special_char_function('=', increase_racket_speed); // +
	register_special_char_function('-', decrease_racket_speed); // -
	register_special_char_function('.', increase_racket_delay); // >
	register_special_char_function(',', decrease_racket_delay); // <
	

	/* GPIO configuration */
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	// switch init
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
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

void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
	  EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void racket_calibrate(void)			//calibrate to 1,run before start
{
	calibrate_mode_on = true;
}

void racket_received_command(void)
{
	switch_on = false;
	racket_mode = 0;
}

void racket_update(void)    //determine whether the motor should run
{
	// calibration mode
	if (calibrate_mode_on) {
		if (!switch_on) {
			motor_set_vel(MOTOR5, 300, OPEN_LOOP);
		}
		else {
			motor_lock(MOTOR5);
			calibrate_mode_on = false;
			calibrated = true;
		}
	}
	// regular mode
	else if ((!switch_on) && calibrated) {
		motor_set_vel(MOTOR5, racket_speed, OPEN_LOOP);
	} else {
			motor_lock(MOTOR5);
	}
}


bool did_receive_command(void)
{
	return !switch_on;
}

u8 get_switch(void){
	return switch_on;
}

u8 get_calibrated(void){
	return calibrated;
}

