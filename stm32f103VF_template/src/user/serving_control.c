#include "serving_control.h"
#include "delay.h"

static bool calibrate_mode_on = false;
static bool calibrated = false;
static bool serving_started = false;
static u32 serving_started_time = 0;
static bool switch_state;
static u8 switch_hit = 0;
static u32 switch_interrupt_trigger_time = 0;

static bool swing_start = false;

static u32 current_speed = 0;

static u16 racket_speed = 1800;
static u16 racket_delay = 10;

// Begin internal functions

static void increase_racket_speed() {
	if(racket_speed < 1800)
		racket_speed += 5;
}

static void decrease_racket_speed() {
	if(racket_speed > 0)
			racket_speed -= 5;
}

static void increase_racket_delay() {
	racket_delay += 5;
}

static void decrease_racket_delay() {
	racket_delay -= 5;
}

static void slow_serve_speed_set(void) {
	racket_speed = 1000;
}

static void mid_serve_speed_set(void) {
	racket_speed = 1400;
}

static void fast_serve_speed_set(void) {
	racket_speed = 1800;
}

static void motor_spin(void) {
	motor_set_vel(MOTOR5, -1000, OPEN_LOOP);
}

static void motor_emergency_stop(void) {
	motor_lock(MOTOR5);
}


static void open_pneumatic(void)
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
}

static void close_pneumatic(void)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_15);
}

static void serving (void){
	// check if pneumatic is closed
	if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15)) {
		open_pneumatic();
		serving_started = true;
		serving_started_time = get_full_ticks();
	}
}

static void serving_calibrate(void)			//calibrate to 1, run before start
{
	calibrate_mode_on = true;
	calibrated = false;
	switch_hit = 0;
}

static void serving_received_command(void)
{
	if (calibrated) {
		swing_start = true;
	}
}

// end internal functions

void serving_init(void)
{
	register_special_char_function('u', serving_received_command);
	register_special_char_function(']', serving_calibrate);
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
	
	close_pneumatic();
}

void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
		// Debounced edge trigger
		if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) != switch_state && get_full_ticks() - switch_interrupt_trigger_time > SWITCH_TIMEOUT) {
			switch_interrupt_trigger_time = get_full_ticks();
			switch_state = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7);
			// Only when switch is being turned on
			if (switch_state) {
				if (calibrate_mode_on) {
					calibrate_mode_on = false;
					calibrated = true;
				} else if (swing_start) {
					swing_start = false;
				}
				motor_lock(MOTOR5);
				current_speed = 0;
				/*
				// do nothing if calibrating
				if (!calibrate_mode_on) {
					// decelerate when hitting the switch again
					motor_set_acceleration(MOTOR5, 1000);
					motor_set_vel(MOTOR5, -20, CLOSE_LOOP);
					current_speed = 20;
				}
				*/
			}
			// Only when switch is being turned off
			else {
			}
		}
	  EXTI_ClearITPendingBit(EXTI_Line7);
	}
}

void serving_update(void)    //determine whether the motor should run
{
	if (swing_start) {
		motor_set_vel(MOTOR5, -racket_speed, OPEN_LOOP);
		current_speed = racket_speed;
	}
	// calibration mode
	if (calibrate_mode_on) {
		calibrated = false;
		motor_set_vel(MOTOR5, -150, OPEN_LOOP);
		current_speed = 150;
	}
	// regular mode
	if (serving_started) {
		if (get_full_ticks() - serving_started_time > racket_delay){
			serving_received_command();
			serving_started = false;
		}
	}
}

u8 serving_get_switch(void){
	return switch_hit;
}

s32 serving_get_calibrated(void){
	return switch_state;
}

s32 serving_get_current(void){
	return switch_hit;
}

s32 serving_get_prev(void){
	return current_speed;
}

u16 serving_get_racket_speed() {
	return racket_speed;
}

u16 serving_get_racket_delay() {
	return racket_delay;
}
