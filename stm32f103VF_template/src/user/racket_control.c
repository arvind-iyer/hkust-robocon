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
static bool switch_stat;
static u8 switch_hit = 0;

static u32 current_speed = 0;

static u16 racket_speed = 1800;
static u16 racket_delay = 264;    //tested best result

//added for upper rackets
static bool up_calibrate_mode_on = false;
static bool up_calibrated = false;
static s32 up_prev_encoder_value = 0;
static s32 up_current_encoder_value = 0;
static s32 up_target_encoder_value = 0;
static s32 up_turn_encoder_value = 0;
static bool up_switch_stat;
static u8 up_switch_hit = 0;
static u32 up_current_speed = 0;
static u16 up_racket_speed = 1800;
static s32 up_switch_encoder_value = 0;

static bool up_switch_state;
static s32 up_switch_interrupt_trigger_time = 0;

// Getters
u16 get_racket_speed() {
	return racket_speed;
}

u16 get_racket_delay() {
	return racket_delay;
}

u8 get_switch(void){
	return switch_hit;
}

s32 get_calibrated(void){
	return racket_delay;
}

s32 get_current(void){
	return current_speed;
}

s32 get_prev(void){
	return prev_encoder_value;
}

u8 get_up_switch(void){
	return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11);
}

// internal functions (special char functions)
static void increase_racket_speed() {
	if(racket_speed < 1800)
		racket_speed += 5;
}

static void decrease_racket_speed() {
	if(racket_speed > 0)
			racket_speed -= 5;
}

static void increase_racket_delay() {
	racket_delay += 1;
}

static void decrease_racket_delay() {
	racket_delay -= 1;
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

static void up_racket_cmd(void)
{
	if (up_calibrated) {
		up_target_encoder_value = up_switch_encoder_value + up_turn_encoder_value + 300;
	}
}

static void up_racket_calibrate (void){
	up_calibrate_mode_on = true;
	up_prev_encoder_value = 0;
	up_current_encoder_value = 0;
	up_turn_encoder_value = 0;
	up_calibrated = false;
	up_switch_hit = 0;
}

static void racket_calibrate(void)			//calibrate to 1, run before start
{
	calibrate_mode_on = true;
	prev_encoder_value = 0;
	current_encoder_value = 0;
	turn_encoder_value = 0;
	calibrated = false;
	switch_hit = 0;
}

static void racket_received_command(void)
{
	if (calibrated) {
		target_encoder_value = current_encoder_value + turn_encoder_value;
	}
}

// interface implementation

void racket_init(void)
{
	register_special_char_function('u', racket_received_command);
	register_special_char_function('[', racket_calibrate);
	register_special_char_function('o', open_pneumatic);
	register_special_char_function('p', close_pneumatic);
	register_special_char_function('l', serving);
	register_special_char_function('b', slow_serve_speed_set);
	register_special_char_function('n', mid_serve_speed_set);
	register_special_char_function('m', fast_serve_speed_set);
	register_special_char_function('h', motor_spin);
	register_special_char_function('g', motor_emergency_stop);
	
	register_special_char_function('k', up_racket_cmd);
	register_special_char_function(';', up_racket_calibrate);
	
	
	register_special_char_function('=', increase_racket_speed); // +
	register_special_char_function('-', decrease_racket_speed); // -
	register_special_char_function('.', increase_racket_delay); // >
	register_special_char_function(',', decrease_racket_delay); // <
	
	switch_stat = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7);
	up_switch_stat = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11);

	// GPIO configuration
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	
	// switch_serving init
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//switch_upper_racket init
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//interrupt for upper racket
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource11);
	
	// EXTI configuration
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	// NVIC configuration
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	// pneumatic init	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	close_pneumatic();
}

void EXTI15_10_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
		// Debounced edge trigger
		if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11) != up_switch_state && get_full_ticks() - up_switch_interrupt_trigger_time > SWITCH_TIMEOUT) {
			up_switch_interrupt_trigger_time = get_full_ticks();
			up_switch_state = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11);
			// Only when switch is being turned on
			if (up_switch_state) {
				up_switch_encoder_value = get_encoder_value(MOTOR6);
			}
			// Only when switch is being turned off
			else {
			}
		}
	  EXTI_ClearITPendingBit(EXTI_Line11);
	}
}

void racket_update(void)    //determine whether the motor should run
{
	if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) != switch_stat) {
		switch_stat = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7);
		if (switch_stat) {
			++switch_hit;
			if (!prev_encoder_value) {
				prev_encoder_value = get_encoder_value(MOTOR5);
			} else if (!current_encoder_value) {
				current_encoder_value = get_encoder_value(MOTOR5);
			}
		}
	}
	// calibration mode
	if (calibrate_mode_on) {
		if (switch_hit < 2) {
			calibrated = false;
			motor_set_vel(MOTOR5, -150, OPEN_LOOP);
			current_speed = 150;
		} else {
			calibrate_mode_on = false;
			calibrated = true;
			turn_encoder_value = current_encoder_value - prev_encoder_value;
			target_encoder_value = current_encoder_value;
			motor_lock(MOTOR5);
			current_speed = 0;
			serving_started_time = 0;
		}
	}
	// regular mode
	else if (calibrated) {
		if (get_encoder_value(MOTOR5) >= target_encoder_value) {
			current_encoder_value = get_encoder_value(MOTOR5);
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

// added for upper rackets

void up_racket_update(void)    //determine whether the motor should run
{
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11) != up_switch_stat) {
		up_switch_stat = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11);
		if (up_switch_stat) {
			++up_switch_hit;
			if (!up_prev_encoder_value) {
				up_prev_encoder_value = get_encoder_value(MOTOR6);
			} else if (!up_current_encoder_value) {
				up_current_encoder_value = get_encoder_value(MOTOR6);
			}
		}
	}
	// calibration mode
	if (up_calibrate_mode_on) {
		if (up_switch_hit < 2) {
			up_calibrated = false;
			motor_set_vel(MOTOR6, -150, OPEN_LOOP);
			up_current_speed = 150;
		} else {
			up_calibrate_mode_on = false;
			up_calibrated = true;
			up_turn_encoder_value = up_current_encoder_value - up_prev_encoder_value;
			up_target_encoder_value = up_current_encoder_value;
		}
	}
	// regular mode
	else if (up_calibrated) {
		if (get_encoder_value(MOTOR6) >= up_target_encoder_value) {
			up_current_encoder_value = get_encoder_value(MOTOR6);
			motor_lock(MOTOR6);
			up_current_speed = 0;
		} else if (get_encoder_value(MOTOR6) <= up_target_encoder_value - up_turn_encoder_value / 2){
			motor_set_vel(MOTOR6, -up_racket_speed, OPEN_LOOP);
			up_current_speed = up_racket_speed;
		} else {
			s32 vel_error = (up_target_encoder_value - get_encoder_value(MOTOR6))* up_racket_speed  / up_turn_encoder_value + 200;
			motor_set_vel(MOTOR6, -vel_error, OPEN_LOOP);
			up_current_speed = vel_error;
		}
	}
}

