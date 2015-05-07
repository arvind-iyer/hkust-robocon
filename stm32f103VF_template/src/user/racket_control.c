#include "racket_control.h"
#include "delay.h"
#include "approx_math.h"
#include "buzzer_song.h"

// calibrate mode static variables
static bool calibrate_mode_on = false;
static bool calibrated = false;
static u8 racket_calibration_mode = 0; // 0 - slow, 1 - fast
static s32 prev_encoder_value = 0;
static s32 current_encoder_value = 0;
static s32 target_encoder_value = 0;
static s32 turn_encoder_value = 0;
static const u16 CALIBRATE_SPEED = 20;
static const u16 SWING_RETURN_MIN_SPEED = 15;

static u16 delay_counter = 0;

// serving static variables
static bool serving_started = false;
static u32 serving_started_time = 0;
static bool switch_stat;
static u8 switch_hit = 0;
static bool serving_now = false;

static u8 switch_trigger_number = 0;

static bool racket_enable = false;

static bool trigger_serve = false;
static bool serve_enabled = false;

static bool sensor_on = false;

static u16 current_speed = 0;

// 1650, 52
static u16 racket_speed = 1500;		//tested best result
static u16 close_loop_racket_speed = 60;
static u16 racket_speed_adjust_time = 0;
static u16 racket_delay = 52; //56;    //tested best result
static u16 racket_delay_adjust_time = 0;

// hitting static variables
static bool hitting_mode_on = false;
static u32 hitting_started_time = 0;
static u16 upper_delay = 0;	// Delay (upper_delay) ms to hit when the upper_hit_delay is called
static const u16 UPPER_HOLD_DELAY  = 500;
static const u16 REENABLE_RACKET_DELAY = 500;

static const u16 ULTRA_UPPER_LIMIT = 1300;
static const u16 ULTRA_LOWER_LIMIT = 50;

// defines for hitting GPIO pins
#define HIT_RACKET_PIN_1 GPIO_Pin_9
#define HIT_RACKET_PIN_2 GPIO_Pin_8
// #define HIT_RACKET_PIN_3 GPIO_Pin_12
// #define HIT_RACKET_PIN_4 GPIO_Pin_13
// #define HIT_RACKET_PIN_5 GPIO_Pin_14

const static u8 HIT_RACKET_NUMBER = 2;

const static uint16_t HIT_RACKET_PINS[HIT_RACKET_NUMBER] = 
		{HIT_RACKET_PIN_1,
		HIT_RACKET_PIN_2,
//		HIT_RACKET_PIN_3,
//		HIT_RACKET_PIN_4,
//		HIT_RACKET_PIN_5
			};
		
const static u16 HIT_Y_DISTANCE = 600; // 600 millimeters

// defines for serving GPIO pins
#define SERVE_RACKET_PIN GPIO_Pin_5
		
#define SERVE_SWITCH_PIN GPIO_Pin_11

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

s32 get_current_speed(void){
	return current_speed;
}

s32 get_turn_encoder_value(void) {
	return turn_encoder_value;
}

s32 get_prev_encoder_value(void){
	return prev_encoder_value;
}

u8 get_up_switch(void){
	return GPIO_ReadInputDataBit(GPIOE, SERVE_SWITCH_PIN);
}

u8 get_switch_trigger_number(void) {
	return switch_trigger_number;
}

bool is_serving(void) {
	return serving_now;
}

// internal functions (special char functions)
static void increase_racket_speed() {
	if(racket_speed < 1800 && get_full_ticks() - racket_speed_adjust_time > 200)
	{
		racket_speed_adjust_time = get_full_ticks();
		racket_speed += 5;
	}
}

static void decrease_racket_speed() {
	if(get_full_ticks() - racket_speed_adjust_time > 200)
	{
		racket_speed_adjust_time = get_full_ticks();
		if (racket_speed > 0)
		{
			racket_speed -= 5;
		}
	}
}

static void increase_racket_delay() {
	static u8 counter = 0;
	++counter;
	if(counter > 10) {
		counter = 0;
		racket_delay += 1;
	}
}

static void decrease_racket_delay() {
	static u8 counter = 0;
	++counter;
	if(counter > 10) {
		counter = 0;
		if(racket_delay > 0) {
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
	if (GPIO_ReadInputDataBit(GPIOE, SERVE_RACKET_PIN) && calibrated && serve_enabled) 
	{
		trigger_serve = true;
		serve_enabled = false;
	}
}

// calibrate the racket
// will automatically turn on fast calibrate once completed
void racket_calibrate(void)			//calibrate to 1, run before start
{
	calibrate_mode_on = true;
	prev_encoder_value = 0;
	current_encoder_value = 0;
//	if (racket_calibration_mode != 1) {
		turn_encoder_value = 0;
//	}
	calibrated = false;
	switch_hit = 0;
	disable_ultrasonic_sensor();
}

void enable_ultrasonic_sensor(void)
{
	sensor_on = true;
}

void disable_ultrasonic_sensor(void)
{
	sensor_on = false;
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
	register_special_char_function('[', racket_calibrate);
	register_special_char_function('k', serving);
	register_special_char_function('l', upper_hit);
	
	register_special_char_function(':', enable_ultrasonic_sensor);
	register_special_char_function(';', disable_ultrasonic_sensor);	
	
//	register_special_char_function('j', racket_received_command);
	
	// debug commands
	register_special_char_function('Y', open_pneumatic);
	register_special_char_function('y', close_pneumatic);
	register_special_char_function('J', open_upper_pneumatic);
	register_special_char_function('H', close_upper_pneumatic);
	register_special_char_function('N', set_fast_calibrate_mode);
	register_special_char_function('M', set_slow_calibrate_mode);
	
	register_special_char_function('=', increase_racket_speed); // +
	register_special_char_function('-', decrease_racket_speed); // -
	register_special_char_function('.', increase_racket_delay); // >
	register_special_char_function(',', decrease_racket_delay); // <
	
	// GPIO configuration
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	// switch_serving init
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = SERVE_SWITCH_PIN;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//interrupt for upper racket
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource12);

	// EXTI configuration
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
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
		GPIO_Init(GPIOD, &GPIO_InitStructure);
	}
	
	close_pneumatic();
	close_upper_pneumatic();

	switch_stat = GPIO_ReadInputDataBit(GPIOE, SERVE_SWITCH_PIN);
	
	motor_set_acceleration(MOTOR5, 1000);
}

// IRQ handler for GPIOE 12
void EXTI15_10_IRQHandler(void)
{
	// check status of GPIOE 12
	if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
		// Switch handling?
	  EXTI_ClearITPendingBit(EXTI_Line12);
	}
}

/**
	* @brief Updates all internal logic of serving rackets
	*/
void racket_update(void)    //determine whether the motor should run
{
	static bool switch_five_tick_delay = false;
	// calibration mode
	if (switch_stat && switch_five_tick_delay == true) {
		++switch_hit;
		switch_five_tick_delay = false;
	}
	if (calibrate_mode_on) {
		// update encoder values when switch is hit
		if(GPIO_ReadInputDataBit(GPIOE, SERVE_SWITCH_PIN) != switch_stat) {
			switch_stat = GPIO_ReadInputDataBit(GPIOE, SERVE_SWITCH_PIN);
			if (switch_stat) {
				switch_five_tick_delay = true;
				++switch_trigger_number;
				if (!prev_encoder_value) {
					prev_encoder_value = get_encoder_value(MOTOR5);
				} else if (!current_encoder_value) {
					current_encoder_value = get_encoder_value(MOTOR5);
				}
			}
		}
		if (switch_hit < 2) {
			calibrated = false;
			motor_set_vel(MOTOR5, -CALIBRATE_SPEED, CLOSE_LOOP);
			current_speed = CALIBRATE_SPEED;
		} else {
			calibrate_mode_on = false;
			calibrated = true;
			turn_encoder_value = current_encoder_value - prev_encoder_value;
			target_encoder_value = current_encoder_value;
			motor_lock(MOTOR5);
			current_speed = 0;
			serving_started_time = 0;
			serve_enabled = true;
		}
	}
	// regular mode
	else if (calibrated) {
		if (trigger_serve) {
			open_pneumatic();
			serving_started = true;
			trigger_serve = false;
		}
		else if (serving_started) {
			++delay_counter;
			if (delay_counter > racket_delay) {
				delay_counter = 0;
				racket_received_command();
				serving_started = false;
				serving_now = true;
				close_pneumatic();
			}
		}
		else if (serving) {
			if (get_encoder_value(MOTOR5) >= target_encoder_value - turn_encoder_value / 8) {
				current_encoder_value = get_encoder_value(MOTOR5);
				motor_lock(MOTOR5);
				current_speed = 0;
				serving_now = false;
			}
			else if (get_encoder_value(MOTOR5) <= target_encoder_value - turn_encoder_value / 2){
				FAIL_MUSIC;
				motor_set_vel(MOTOR5, -racket_speed, OPEN_LOOP);
				current_speed = racket_speed;
			} else {
				s32 vel_error = (target_encoder_value - get_encoder_value(MOTOR5)) * racket_speed  / turn_encoder_value;
				motor_set_vel(MOTOR5, -vel_error, OPEN_LOOP);
				current_speed = vel_error;
			}
		}
		else if (!serving) {
			motor_lock(MOTOR5);
			current_speed = 0;
			serving_now = false;
			SUCCESSFUL_MUSIC;
		}
	}
}

/**
	* @brief Hit the upper racket after 'delay' ms
	* @param The delay time (in millisecond)
	*/
void upper_hit_delay(u16 delay)
{
	if (racket_enable){
		hitting_mode_on = delay == 0 ? true : false;
		hitting_started_time = get_full_ticks();
		upper_delay = delay;
	}
}

// added for upper rackets
void upper_hit(void){
	upper_hit_delay(0);
}

void open_upper_pneumatic(void)
{
	for (u8 i = 0; i < HIT_RACKET_NUMBER; ++i) {
		GPIO_SetBits(GPIOD, HIT_RACKET_PINS[i]);
	}
}

void close_upper_pneumatic(void)
{
	for (u8 i = 0; i < HIT_RACKET_NUMBER; ++i) {
		GPIO_ResetBits(GPIOD, HIT_RACKET_PINS[i]);
	}
}

typedef struct {
	u16 y_distance;
	u16 sense_time;
} ultrasonic_distance_data;

typedef struct {
	int dy;
	int dt;
} ultrasonic_velocity_data;

typedef struct {
	int acceleration;
	int number_of_averages;
} ultrasonic_accel_data;

static ultrasonic_distance_data ultra_data = {0, 0};
static ultrasonic_distance_data reset_ultra_data = {0, 0};

const static u8 shuttle_acceleration_scale_factor = 100; // acceleration scaled by 100
static ultrasonic_accel_data shuttle_acceleration = {0, 0};

static ultrasonic_velocity_data prev_shuttle_velocity = {0, 0};
static int global_delay = 0;
static u16 sensor_last_sensed_time = 0;

s32 sensor_calculate_delay(u16 second_y_distance) {
	// calculate the current velocity
	if (ultra_data.y_distance == 0 || ultra_data.sense_time == 0) {
		ultra_data.y_distance = second_y_distance;
		ultra_data.sense_time = get_full_ticks();
		return -1;
	}
	int dy2 = second_y_distance - ultra_data.y_distance;
	int dt2 = get_full_ticks() - ultra_data.sense_time;
	
	bool average_calculated = false;
	// use velocity to find acceleration, only if a previous shuttle velocity exists
	buzzer_control_note(5, 50, NOTE_D, 7);
	
	if (prev_shuttle_velocity.dy != 0 && prev_shuttle_velocity.dt != 0) {
		average_calculated = true;
		// (v2 - v1) / (t2 - t1)
		int acceleration = (dy2 * prev_shuttle_velocity.dt - prev_shuttle_velocity.dy * dt2) * shuttle_acceleration_scale_factor / (prev_shuttle_velocity.dt * dt2 * prev_shuttle_velocity.dt * dt2);
		// weighted average acceleration
		shuttle_acceleration.acceleration = (shuttle_acceleration.acceleration * shuttle_acceleration.number_of_averages + acceleration) / (shuttle_acceleration.number_of_averages + 1);
		++shuttle_acceleration.number_of_averages;
	}
	// assign as previous velocity
	prev_shuttle_velocity.dy = dy2;
	prev_shuttle_velocity.dt = dt2;
	
	// if acceleration is 0, just return invalid number here
	if (!average_calculated) {
		return -1;
	}
	
	// use delta y = 1/2 * a * t^2 + vo * t extrapolation, solve for t
	// equation is t = (sqrt(2 * a * (y - y0) + (v0)^2) - v0) / a
	// v0 is dy2/dt2
	
	int t = (Sqrt( 2 * shuttle_acceleration.acceleration * (HIT_Y_DISTANCE - second_y_distance) / shuttle_acceleration_scale_factor + Sqr(dy2) / Sqr(dt2) ) - dy2 / dt2) * shuttle_acceleration_scale_factor / shuttle_acceleration.acceleration;
	
	if (t > 0) {
		return t;
	} else {
		return 0;
	}
}

/**
	* @brief Checks if sensors detect the ball
	*/

typedef struct
{
	u8 first, second;
} ut_pairs;

void up_racket_sensor_check(void)
{
	if (!sensor_on) return;
	/*
	const static u16 SENSOR_TIMEOUT = 300;
	if (get_full_ticks() - sensor_last_sensed_time > SENSOR_TIMEOUT) {
		// reset all the variables
		shuttle_acceleration.acceleration = 0;
		shuttle_acceleration.number_of_averages = 0;
		prev_shuttle_velocity.dy = 0;
		prev_shuttle_velocity.dt = 0;
		ultra_data.y_distance = 0;
		ultra_data.sense_time = 0;
	}
	
	// check all sensors for distance within threshold
	for (u8 id = 0; id < US_DEVICE_COUNT; ++id) {
		if (us_get_distance(id) >= 50 && us_get_distance(id) <= 1500) {
			sensor_last_sensed_time = get_full_ticks();
			int delay = sensor_calculate_delay(us_get_distance(id));
			ultra_data.y_distance = us_get_distance(id);
			ultra_data.sense_time = get_full_ticks();
			global_delay = delay;
			if (delay < 0) {
				// do nothing
			}
			else if (delay < 100) {
				upper_hit_delay(delay);
			}
			// make sure only one sensor's distance is used
			break;
		}
	}
	*/
	
	static u8 previous_detection = 0;
	static ut_pairs pairs[] = { {6,3}, {3,4}, {4,1}, {1,2}, {2,7}, {7,0}, {0,8} };
	static u8 pair_length = 7;
	u8 tmp_detection = 0;
	
	// If any sensors sense 
	//for (u8 id = 0; id < US_DEVICE_COUNT; ++id) {
	//	if (us_get_distance(id) >= 50 && us_get_distance(id) <= 1200) {
	//		++tmp_detection;
	//	}
	//}
	
	for (u8 pair = 0; pair < pair_length; ++pair) {
		u16 distance1 = us_get_distance(pairs[pair].first);
		u16 distance2 = us_get_distance(pairs[pair].second);
		if (distance1 >= ULTRA_LOWER_LIMIT && distance1 <= ULTRA_UPPER_LIMIT
			&& distance2 >= ULTRA_LOWER_LIMIT && distance2 <= ULTRA_UPPER_LIMIT)
		{
			buzzer_control_note(5, 50, NOTE_E, 7);
			tmp_detection = 1;
			break;
		}
	}
		
	if (previous_detection == 0 && tmp_detection && !hitting_mode_on) {
		buzzer_control_note(5, 50, NOTE_C, 7);
		upper_hit_delay(100);
	}
	previous_detection = tmp_detection;
	
}

int get_global_delay(void)
{
	return shuttle_acceleration.acceleration;
}

/**
	* @brief Updates all internal logic of upper rackets
	*/
void up_racket_update(void)
{
	if (racket_enable == true && hitting_mode_on == false && get_full_ticks() >= hitting_started_time + upper_delay) {
		hitting_mode_on = true;
	}
	
	if(racket_enable == true && hitting_mode_on == true){
		open_upper_pneumatic();
		racket_enable = false;
	}
	
	if (get_full_ticks() > hitting_started_time + UPPER_HOLD_DELAY + upper_delay){
		hitting_mode_on = false;
		close_upper_pneumatic();
	}
	
	if (get_full_ticks() > hitting_started_time + UPPER_HOLD_DELAY + upper_delay + REENABLE_RACKET_DELAY){
		racket_enable = true;
		hitting_started_time = 0;
	}
}
