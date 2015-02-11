#include "racket_control.h"
#include "delay.h"

static bool calibrate_mode_on = false;
static bool calibrated = false;
static s32 prev_encoder_value = 0;
static s32 current_encoder_value = 0;
static s32 target_encoder_value = 0;
static s32 turn_encoder_value = 0;
static bool serving_started = false;
static u32 serving_started_time = 0;
static bool switch_state;
static u8 switch_hit = 0;
static u32 switch_interrupt_trigger_time = 0;

static u32 current_speed = 0;

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

void slow_serve_speed_set(void) {
	racket_speed = 1000;
}

void mid_serve_speed_set(void) {
	racket_speed = 1400;
}

void fast_serve_speed_set(void) {
	racket_speed = 1800;
}

void motor_spin(void) {
	motor_set_vel(MOTOR5, -1000, OPEN_LOOP);
}

void motor_emergency_stop(void) {
	motor_lock(MOTOR5);
}

void racket_init(void)
{
	register_special_char_function('u', racket_received_command);
	register_special_char_function(']', racket_calibrate);
	register_special_char_function('y', open_pneumatic);
	register_special_char_function('j', close_pneumatic);
	register_special_char_function('o', serving);
	register_special_char_function('b', slow_serve_speed_set);
	register_special_char_function('n', mid_serve_speed_set);
	register_special_char_function('m', fast_serve_speed_set);
	register_special_char_function('h', motor_spin);
	register_special_char_function('g', motor_emergency_stop);
	
	register_special_char_function('=', increase_racket_speed); // +
	register_special_char_function('-', decrease_racket_speed); // -
	register_special_char_function('.', increase_racket_delay); // >
	register_special_char_function(',', decrease_racket_delay); // <
	
	switch_state = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7);

	// GPIO configuration
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
	
	// EXTI configuration
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	// NVIC configuration
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
		if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) != switch_state && get_full_ticks() - switch_interrupt_trigger_time > SWITCH_TIMEOUT) {
			switch_interrupt_trigger_time = get_full_ticks();
			switch_state = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7);
			if (switch_state) {
				switch_hit++;
				if (!prev_encoder_value) {
					prev_encoder_value = get_encoder_value(MOTOR5);
				} else if (!current_encoder_value) {
					current_encoder_value = get_encoder_value(MOTOR5);
				}
			}
		}
		if (calibrate_mode_on && switch_hit == 2) {
			calibrate_mode_on = false;
			calibrated = true;
			turn_encoder_value = current_encoder_value - prev_encoder_value;
			target_encoder_value = current_encoder_value;
		}
	  EXTI_ClearITPendingBit(EXTI_Line7);
	}
}

void racket_calibrate(void)			//calibrate to 1, run before start
{
	calibrate_mode_on = true;
	prev_encoder_value = 0;
	current_encoder_value = 0;
	turn_encoder_value = 0;
	calibrated = false;
	switch_hit = 0;
}

void racket_received_command(void)
{
	if (calibrated) {
		target_encoder_value = current_encoder_value + turn_encoder_value;
	}
}

void racket_update(void)    //determine whether the motor should run
{
	// calibration mode
	if (calibrate_mode_on) {
		calibrated = false;
		motor_set_vel(MOTOR5, -150, OPEN_LOOP);
		current_speed = 150;
	}
	// regular mode
	else if (calibrated) {
		if (get_encoder_value(MOTOR5) >= target_encoder_value) {
			current_encoder_value = get_encoder_value(MOTOR5);
			// motor_set_acceleration(MOTOR5, 1000);
			// motor_set_vel(MOTOR5, 0, CLOSE_LOOP);
			motor_lock(MOTOR5);
			current_speed = 0;
		} else if (get_encoder_value(MOTOR5) <= target_encoder_value - turn_encoder_value / 2){
			motor_set_vel(MOTOR5, -racket_speed, OPEN_LOOP);
			current_speed = racket_speed;
		} else {
			s32 vel_error = (target_encoder_value - get_encoder_value(MOTOR5))* racket_speed  / turn_encoder_value + 200;
			motor_set_vel(MOTOR5, -vel_error, OPEN_LOOP);
			current_speed = vel_error;
		}
	}
	if (serving_started) {
		if (get_full_ticks() - serving_started_time > racket_delay){
			racket_received_command();
			serving_started = false;
		}
	}
}


bool did_receive_command(void)
{
	return false;
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
	return switch_hit;
}

s32 get_calibrated(void){
	return racket_delay;
}

s32 get_current(void){
	return switch_hit;
}

s32 get_prev(void){
	return prev_encoder_value;
}
