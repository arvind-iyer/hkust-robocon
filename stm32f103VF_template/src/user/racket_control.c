#include "racket_control.h"
#include "delay.h"

static bool switch_on = false;
static bool calibrate_mode_on = false;
static bool calibrated = false;
static bool interrupt_triggered = false;
static u32 interrupt_triggered_time;
//static u32 prev_encoder_value = 0;
//static u32 current_encoder_distance = 0;
static bool serving_started = false;
static u32 serving_started_time = 0;

void racket_init(void)
{
	register_special_char_function('u', racket_received_command);
	register_special_char_function(']', racket_calibrate);
	register_special_char_function('y', open_pneumatic);
	register_special_char_function('j', close_pneumatic);
	register_special_char_function('o', serving);

	/* GPIO configuration */
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	// switch init
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	// pneumatic init	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
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

void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
		interrupt_triggered_time = get_full_ticks();
		interrupt_triggered = true;
	  EXTI_ClearITPendingBit(EXTI_Line7);
	}
}

void racket_calibrate(void)			//calibrate to 1,run before start
{
	calibrate_mode_on = true;
}

void racket_received_command(void)
{
	switch_on = false;
}

void racket_update(void)    //determine whether the motor should run
{
	
	if (interrupt_triggered && get_full_ticks() - interrupt_triggered_time > 20) {
		if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7)) {
			switch_on = true;
		} else {
			switch_on = false;
		}
		interrupt_triggered = false;
	}
	// calibration mode
	if (calibrate_mode_on) {
		if (!switch_on) {
			motor_set_vel(MOTOR5, 300, OPEN_LOOP);
		}
		else {
//			motor_set_acceleration(MOTOR5, 1000);
//			motor_set_vel(MOTOR5, 0, CLOSE_LOOP);
			motor_lock(MOTOR5);
			calibrate_mode_on = false;
			calibrated = true;
		}
	}
	// regular mode
	else if ((!switch_on) && calibrated) {
		motor_set_vel(MOTOR5, 800, OPEN_LOOP);
	} else {
//		motor_set_acceleration(MOTOR5, 1000);
//		motor_set_vel(MOTOR5, 0, CLOSE_LOOP);
			motor_lock(MOTOR5);
	}
	if (serving_started) {
		if (get_full_ticks() - serving_started_time > MOTOR_OPEN_DELAY){
			racket_received_command();
			serving_started = false;
		}
	}
}


bool did_receive_command(void)
{
	return !switch_on;
}

void open_pneumatic(void)
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
}

void close_pneumatic(void)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_15);
}

void serving (void){
	// check if pneumatic is closed
	if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15)) {
		open_pneumatic();
		serving_started = true;
		serving_started_time = get_full_ticks();
	}
}

u8 get_switch(void){
	return switch_on;
}

u8 get_calibrated(void){
	return calibrated;
}

