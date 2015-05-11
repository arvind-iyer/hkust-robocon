#include "racket_control.h"
#include "delay.h"
#include "approx_math.h"
#include "buzzer_song.h"
#include "send_debug_data.h"

// current speed of serving racket
static u16 current_speed = 0;

// calibrate mode static variables
typedef enum { SERVING, CALIBRATING, LOCKED } serving_racket_mode;

static serving_racket_mode current_racket_mode = LOCKED;

static bool racket_has_been_calibrated = false;
static const u16 CALIBRATE_SPEED = 20;

// serving static variables
static bool racket_enable = false;

// 1650, 52
// 1500, 56
static u16 racket_speed = 1400;		//tested best result
static u16 racket_speed_adjust_time = 0;
static u16 racket_delay = 56; //56;    //tested best result

// hitting static variables
static bool hitting_mode_on = false;
static u32 hitting_started_time = 0;
static u16 upper_delay = 0;	// Delay (upper_delay) ms to hit when the upper_hit_delay is called
static const u16 UPPER_HOLD_DELAY  = 500;
static const u16 REENABLE_RACKET_DELAY = 500;

// ultrasonic variables
static bool sensor_on = false;
static const u16 ULTRA_UPPER_LIMIT = 1300;
static const u16 ULTRA_LOWER_LIMIT = 50;
const static u16 HIT_Y_DISTANCE = 600; // 600 millimeters

static u16 switch_last_triggered_time = 0;
static u8 current_switch_status = 0;

// defines for hitting GPIO pins
#define HIT_RACKET_PIN_1 GPIO_Pin_8
#define HIT_RACKET_PIN_2 GPIO_Pin_9
// #define HIT_RACKET_PIN_3 GPIO_Pin_12
// #define HIT_RACKET_PIN_4 GPIO_Pin_13
// #define HIT_RACKET_PIN_5 GPIO_Pin_14

const static uint16_t HIT_RACKET_PINS[] =
		{HIT_RACKET_PIN_1,
		HIT_RACKET_PIN_2,
//		HIT_RACKET_PIN_3,
//		HIT_RACKET_PIN_4,
//		HIT_RACKET_PIN_5
};

// defines for serving GPIO pins
#define SERVE_RACKET_PIN GPIO_Pin_5
#define SERVE_SWITCH_PIN GPIO_Pin_11

// Getters
u16 racket_get_racket_speed() {
	return racket_speed;
}

u16 racket_get_racket_delay() {
	return racket_delay;
}

u8 racket_get_switch(void){
	return current_switch_status;
}

s32 racket_get_calibration_status(void){
	return racket_has_been_calibrated;
}

s32 racket_get_current_speed(void){
	return current_speed;
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

void racket_increase_racket_delay() {
	static u8 counter = 0;
	++counter;
	if(counter > 20) {
		counter = 0;
		racket_delay += 1;
	}
}

void racket_decrease_racket_delay() {
	static u8 counter = 0;
	++counter;
	if(counter > 20) {
		counter = 0;
		if(racket_delay > 0) {
			racket_delay -= 1;
		}
	}
}

// exposed functions for xbc_button class

void racket_open_serve_pneumatic(void)
{
	GPIO_ResetBits(GPIOE, SERVE_RACKET_PIN);
}

void racket_close_serve_pneumatic(void)
{
	GPIO_SetBits(GPIOE, SERVE_RACKET_PIN);
}

void racket_trigger_serving(void){
	// check if pneumatic is closed before serving
	if (GPIO_ReadInputDataBit(GPIOE, SERVE_RACKET_PIN) && current_racket_mode == LOCKED && racket_has_been_calibrated)
	{
		current_racket_mode = SERVING;
		racket_has_been_calibrated = false;
	}
}

// calibrate the racket
void racket_calibrate(void)
{
    if (current_racket_mode == LOCKED) {
				current_racket_mode = CALIBRATING;
        racket_has_been_calibrated = false;
        motor_set_vel(MOTOR5, -CALIBRATE_SPEED, CLOSE_LOOP);
        disable_ultrasonic_sensor();
    }
}

void enable_ultrasonic_sensor(void)
{
	sensor_on = true;
}

void disable_ultrasonic_sensor(void)
{
	sensor_on = false;
}

// interface implementation

void racket_init(void)
{
	register_special_char_function('[', racket_calibrate);
	register_special_char_function('k', racket_trigger_serving);
	register_special_char_function('l', racket_upper_hit);
	
	register_special_char_function(':', enable_ultrasonic_sensor);
	register_special_char_function(';', disable_ultrasonic_sensor);	
		
	// debug commands
	register_special_char_function('Y', racket_open_serve_pneumatic);
	register_special_char_function('y', racket_close_serve_pneumatic);
	register_special_char_function('J', racket_open_upper_pneumatic);
	register_special_char_function('H', racket_close_upper_pneumatic);
	
	register_special_char_function('=', increase_racket_speed); // +
	register_special_char_function('-', decrease_racket_speed); // -
	register_special_char_function('.', racket_increase_racket_delay); // >
	register_special_char_function(',', racket_decrease_racket_delay); // <
	
	// GPIO configuration
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	// switch_serving init
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = SERVE_SWITCH_PIN;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//interrupt for switch
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource11);

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
	for (u8 i = 0; i < sizeof(HIT_RACKET_PINS) / sizeof(HIT_RACKET_PINS[0]); ++i) {
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin = HIT_RACKET_PINS[i];
		GPIO_Init(GPIOD, &GPIO_InitStructure);
	}
	
	racket_close_serve_pneumatic();
	racket_close_upper_pneumatic();

	current_switch_status = GPIO_ReadInputDataBit(GPIOE, SERVE_SWITCH_PIN);
	
	motor_set_acceleration(MOTOR5, 1000);
}

// IRQ handler for GPIOE 12
void EXTI15_10_IRQHandler(void)
{
	// check status of GPIOE 11
	if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
		// debounce the switch
        if (get_full_ticks() - switch_last_triggered_time > SWITCH_TIMEOUT) {
            switch_last_triggered_time = get_full_ticks();
            if (current_racket_mode == CALIBRATING) {
                // check for edge trigger from 0 to 1
                if (GPIO_ReadInputDataBit(GPIOE, SERVE_SWITCH_PIN) > current_switch_status) {
                    motor_lock(MOTOR5);
                    current_racket_mode = LOCKED;
                    racket_has_been_calibrated = true;
                }
            } else if (current_racket_mode == SERVING) {
                // check for edge trigger from 0 to 1
                if (GPIO_ReadInputDataBit(GPIOE, SERVE_SWITCH_PIN) > current_switch_status) {
                    motor_lock(MOTOR5);
                    current_racket_mode = LOCKED;
                }
            }
            current_switch_status = GPIO_ReadInputDataBit(GPIOE, SERVE_SWITCH_PIN);
        }
        EXTI_ClearITPendingBit(EXTI_Line11);
	}
}

/**
	* @brief Updates all internal logic of serving rackets
	*/
void racket_update(void)    //determine whether the motor should run
{
    static int delay_counter = 0;
    if (current_racket_mode == SERVING) {
        // delay logic
        if (delay_counter < racket_delay) {
						racket_open_serve_pneumatic();
            ++delay_counter;
            motor_lock(MOTOR5);
            current_speed = 0;
        } else {
						racket_close_serve_pneumatic();
            motor_set_vel(MOTOR5, -racket_speed, OPEN_LOOP);
            current_speed = racket_speed;
        }
    } else if (current_racket_mode == LOCKED) {
        motor_lock(MOTOR5);
        current_speed = 0;
        delay_counter = 0;
    }
}

/**
	* @brief Hit the upper racket after 'delay' ms
	* @param The delay time (in millisecond)
	*/
void racket_upper_hit_delay(u16 delay)
{
	if (racket_enable){
		hitting_mode_on = delay == 0 ? true : false;
		hitting_started_time = get_full_ticks();
		upper_delay = delay;
	}
}

// added for upper rackets
void racket_upper_hit(void){
	racket_upper_hit_delay(0);
}

void racket_open_upper_pneumatic(void)
{
	for (u8 i = 0; i < sizeof(HIT_RACKET_PINS) / sizeof(HIT_RACKET_PINS[0]); ++i) {
		GPIO_SetBits(GPIOD, HIT_RACKET_PINS[i]);
	}
}

void racket_close_upper_pneumatic(void)
{
	for (u8 i = 0; i < sizeof(HIT_RACKET_PINS) / sizeof(HIT_RACKET_PINS[0]); ++i) {
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
		racket_upper_hit_delay(100);
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
		racket_open_upper_pneumatic();
		racket_enable = false;
	}
	
	if (get_full_ticks() > hitting_started_time + UPPER_HOLD_DELAY + upper_delay){
		hitting_mode_on = false;
		racket_close_upper_pneumatic();
	}
	
	if (get_full_ticks() > hitting_started_time + UPPER_HOLD_DELAY + upper_delay + REENABLE_RACKET_DELAY){
		racket_enable = true;
		hitting_started_time = 0;
	}
}
