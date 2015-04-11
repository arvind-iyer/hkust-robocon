#include "racket_control.h"
#include "delay.h"

// calibrate mode static variables
static bool calibrate_mode_on = false;
static bool calibrated = false;
static u8 racket_calibration_mode = 0; // 0 - slow, 1 - fast
static s32 prev_encoder_value = 0;
static s32 current_encoder_value = 0;
static s32 target_encoder_value = 0;
static s32 turn_encoder_value = 0;

// serving static variables
static bool serving_started = false;
static u32 serving_started_time = 0;
static bool switch_stat;
static u8 switch_hit = 0;

static u16 current_speed = 0;

static u16 racket_speed = 1780;		//tested best result
static u16 racket_speed_adjust_time = 0;
static u16 racket_delay = 321;    //tested best result
static u16 racket_delay_adjust_time = 0;

// hitting static variables
static bool hitting_mode_on = false;
static u32 hitting_started_time = 0;
static u16 upper_delay  = 500;

static bool racket_emergency_stop = false;

// defines for hitting GPIO pins
#define HIT_RACKET_PIN_1 GPIO_Pin_10
#define HIT_RACKET_PIN_2 GPIO_Pin_11
#define HIT_RACKET_PIN_3 GPIO_Pin_12
#define HIT_RACKET_PIN_4 GPIO_Pin_13
#define HIT_RACKET_PIN_5 GPIO_Pin_14

const static uint16_t HIT_RACKET_PINS[5] = 
		{HIT_RACKET_PIN_1,
		HIT_RACKET_PIN_2,
		HIT_RACKET_PIN_3,
		HIT_RACKET_PIN_4,
		HIT_RACKET_PIN_5};
const static u8 HIT_RACKET_NUMBER = 5;

// defines for serving GPIO pins
#define SERVE_RACKET_PIN GPIO_Pin_15

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
	if(racket_speed < 1800 && get_full_ticks() - racket_speed_adjust_time > 80)
	{
		racket_speed_adjust_time = get_full_ticks();
		racket_speed += 5;
	}
}

static void decrease_racket_speed() {
	if(racket_speed > 0 && get_full_ticks() - racket_speed_adjust_time > 80)
	{
		racket_speed_adjust_time = get_full_ticks();
		racket_speed -= 5;
	}
}

static void increase_racket_delay() {
	if(get_full_ticks() - racket_delay_adjust_time > 80) {
		racket_delay_adjust_time = get_full_ticks();
		racket_delay += 1;
	}
}

static void decrease_racket_delay() {
	if(get_full_ticks() - racket_delay_adjust_time > 80) {
		if(racket_delay > 0) {
			racket_delay_adjust_time = get_full_ticks();
			racket_delay -= 1;
		}
	}
}

// exposed functions for xbc_button class

void open_pneumatic(void)
{
	GPIO_ResetBits(GPIOE, SERVE_RACKET_PIN);
}

void close_pneumatic(void)
{
	GPIO_SetBits(GPIOE, SERVE_RACKET_PIN);
}

void serving(void){
	// check if pneumatic is closed before serving
	if (GPIO_ReadInputDataBit(GPIOE, SERVE_RACKET_PIN)) {
		open_pneumatic();
		serving_started = true;
		serving_started_time = get_full_ticks();
	}
}

// calibrate the racket
// will automatically turn on fast calibrate once completed
void racket_calibrate(void)			//calibrate to 1, run before start
{
	calibrate_mode_on = true;
	prev_encoder_value = 0;
	current_encoder_value = 0;
	if (racket_calibration_mode != 1) {
		turn_encoder_value = 0;
	}
	calibrated = false;
	switch_hit = 0;
}

static void racket_received_command(void)
{
	if (calibrated) {
		target_encoder_value = current_encoder_value + turn_encoder_value;
	}
}

static void set_fast_calibrate_mode(void)
{
	if (turn_encoder_value) {
		racket_calibration_mode = 1;
	}
}

static void set_slow_calibrate_mode(void)
{
	racket_calibration_mode = 0;
}

// interface implementation

void racket_init(void)
{
	register_special_char_function('u', racket_received_command);
	register_special_char_function('[', racket_calibrate);
	register_special_char_function('o', open_pneumatic);
	register_special_char_function('p', close_pneumatic);
	register_special_char_function('l', serving);
	register_special_char_function('n', set_fast_calibrate_mode);
	register_special_char_function('m', set_slow_calibrate_mode);
	register_special_char_function('k', upper_hit);
	register_special_char_function('j', open_upper_pneumatic);
	register_special_char_function('h', close_upper_pneumatic);


//	register_special_char_function(';', up_racket_calibrate);
	
	
	register_special_char_function('=', increase_racket_speed); // +
	register_special_char_function('-', decrease_racket_speed); // -
	register_special_char_function('.', increase_racket_delay); // >
	register_special_char_function(',', decrease_racket_delay); // <
	
	switch_stat = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7);

	// GPIO configuration
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	
	// switch_serving init
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	// init external gpio for  
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
	
	// serving pneumatic init	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = SERVE_RACKET_PIN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// hitting pneumatic init
	for (u8 i = 0; i < HIT_RACKET_NUMBER; ++i) {
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin = HIT_RACKET_PINS[i];
		GPIO_Init(GPIOE, &GPIO_InitStructure);
	}
	
	close_pneumatic();
	close_upper_pneumatic();

}

// IRQ handler for GPIOE 10 to 15
//void EXTI15_10_IRQHandler(void)
//{
//	// check status of GPIOE 11
//	if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
//		// add pneumatic code here?	
//	  EXTI_ClearITPendingBit(EXTI_Line11);
//	}
//}

void racket_update(void)    //determine whether the motor should run
{
	// update encoder values when switch is hit
	if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) != switch_stat) {
		switch_stat = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7);
		if (switch_stat && racket_calibration_mode == 0) {
			++switch_hit;
			if (!prev_encoder_value) {

				prev_encoder_value = get_encoder_value(MOTOR5);
			} else if (!current_encoder_value) {
				current_encoder_value = get_encoder_value(MOTOR5);

			}
		} else if (switch_stat && racket_calibration_mode == 1) {
			++switch_hit;
			current_encoder_value = get_encoder_value(MOTOR5);
		}
	}
	// calibration mode
	if (calibrate_mode_on) {
		if (racket_calibration_mode == 1) {
			if (switch_hit < 1) {
				calibrated = false;
				motor_set_vel(MOTOR5, -12, CLOSE_LOOP);
				current_speed = 12;
			} else {
				calibrate_mode_on = false;
				calibrated = true;
				motor_lock(MOTOR5);
				target_encoder_value = current_encoder_value;
				current_speed = 0;
				serving_started_time = 0;
			}
		} else if (switch_hit < 2) {
			calibrated = false;
			motor_set_vel(MOTOR5, -12, CLOSE_LOOP);
			current_speed = 12;
		} else {
			calibrate_mode_on = false;
			calibrated = true;
			turn_encoder_value = current_encoder_value - prev_encoder_value;
			target_encoder_value = current_encoder_value;
			motor_lock(MOTOR5);
			current_speed = 0;
			serving_started_time = 0;
			racket_calibration_mode = 1;
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
			close_pneumatic();
		}
	}
	
	// emergency stop override
	if (racket_emergency_stop) {
		motor_lock(MOTOR5);
	}
}

// added for upper rackets
void upper_hit(void){
		hitting_mode_on = true;
		hitting_started_time = get_full_ticks();
}
void open_upper_pneumatic(void)
{
	for (u8 i = 0; i < HIT_RACKET_NUMBER; ++i) {
		GPIO_SetBits(GPIOE, HIT_RACKET_PINS[i]);
	}
}

void close_upper_pneumatic(void)
{
	for (u8 i = 0; i < HIT_RACKET_NUMBER; ++i) {
		GPIO_ResetBits(GPIOE, HIT_RACKET_PINS[i]);
	}
}

void up_racket_sensor_check(void)
{
	static u16 sensor_full_ticks = 0;
	static u8 previous_detection = 0;
	if (sensor_full_ticks) {
		if (get_full_ticks() - sensor_full_ticks > UPPER_RACKET_SENSE_DELAY) {
			upper_hit();
			buzzer_control_note(5, 50, NOTE_C, 7);
			sensor_full_ticks = 0;
		}
		return;
	}
	
	
	u8 tmp_detection = 0;
	// If any sensors sense 
	for (u8 id = 0; id < US_DEVICE_COUNT; ++id) {
		if (us_get_distance(id) >= 50 && us_get_distance(id) <= 1000) {
			tmp_detection = 1;
		}
	}
	
	if (previous_detection == 0 && tmp_detection == 1 && !hitting_mode_on) {
		sensor_full_ticks = get_full_ticks();
	}
	previous_detection = tmp_detection;
}

void up_racket_update(void)
{
	if(hitting_mode_on == true){
		open_upper_pneumatic();
	}
	if (get_full_ticks() - hitting_started_time > upper_delay){
		hitting_mode_on = false;
		close_upper_pneumatic();
	}
}
