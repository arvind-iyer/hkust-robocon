#include "racket_control.h"
#include "buzzer_song.h"
#include "delay.h"

static u32 current_time = 0;
static bool low_switch = 0;
static bool high_switch = 0;

// Lower racket variables
static u8   low_racket_mode = 0;
static s32  low_racket_speed = 240;
static bool request_low_racket_move = 0;
static u32  request_low_racket_move_time = 0;

// Higher racket variables
static u8   high_racket_mode = 0;
static s32  high_racket_speed = 240;
static bool request_high_racket_move = 0;
static u32  request_high_racket_move_time = 0;


void low_racket_move(void) {
	request_low_racket_move = 1;
	request_low_racket_move_time = get_full_ticks();
}

void low_racket_standby(void) {
	low_racket_mode = 2;
	request_low_racket_move = 1;
	request_low_racket_move_time = get_full_ticks();
}

void high_racket_move(void) {
	request_high_racket_move = 1;
	request_high_racket_move_time = get_full_ticks();
}


void sensor_init(void) {
}

void racket_init(void) {
	register_special_char_function('/', low_racket_move);
	register_special_char_function('.', low_racket_standby);

	register_special_char_function('m', high_racket_move);
	
	/* GPIO configuration */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void sensor_update(void) {
	
}

void racket_update(void) {
	// When B1 and B2 are pressed, their values are 1.
	low_switch = GPIO_ReadInputDataBit(GPIOE, Switch_B1_Pin);
	high_switch = GPIO_ReadInputDataBit(GPIOE, Switch_B2_Pin);
	
	low_racket_update();
	high_racket_update();
}


/**
	* @brief Move or stop the lower racket according to other variables. ( Called by racket_update() )
  */
void low_racket_update() {
	current_time = get_full_ticks();
	
	if (low_racket_mode == 0) {
		low_racket_speed = 240;
		if (low_switch == 1 && (current_time - request_low_racket_move_time > 500) ) {
			low_racket_mode = 1;
			request_low_racket_move = 0;
		}
	} else if (low_racket_mode == 1) {
		low_racket_speed = LOW_RACKET_HIT_SPEED;
		if (low_switch == 1 && (current_time - request_low_racket_move_time > 50) ) {
			request_low_racket_move = 0;
		}
	} else if (low_racket_mode == 2) {
		low_racket_speed = 500;
		if (current_time - request_low_racket_move_time > 300) {
			low_racket_mode = 1;
			request_low_racket_move = 0;
		}
	} else {
		low_racket_speed = 240;
		request_low_racket_move = 0;
	}
	
	if(request_low_racket_move == 1) {
		motor_set_vel(LOW_RACKET_MOTOR, low_racket_speed, OPEN_LOOP);
	} else {
		motor_lock(LOW_RACKET_MOTOR);
	}
}

/**
	* @brief Move or stop the higher racket according to other variables. ( Called by racket_update() )
  */
void high_racket_update() {
	current_time = get_full_ticks();	
	
	if (high_racket_mode == 0) {
		high_racket_speed = 240;
		if (high_switch == 1 && (current_time - request_high_racket_move_time > 100) ) {
			high_racket_mode = 1;
			request_high_racket_move = 0;
		}
	} else if (high_racket_mode == 1) {
		high_racket_speed = HIGH_RACKET_HIT_SPEED;
		if (high_switch == 1 && (current_time - request_high_racket_move_time > 50) ) {
			request_high_racket_move = 0;
		}
	} else if (high_racket_mode == 2) {
		high_racket_speed = 500;
		if (current_time - request_high_racket_move_time > 300) {
			high_racket_mode = 1;
			request_high_racket_move = 0;
		}
	} else {
		high_racket_speed = 240;
		request_high_racket_move = 0;
	}
	
	if(request_high_racket_move == 1) {
		motor_set_vel(HIGH_RACKET_MOTOR, high_racket_speed, OPEN_LOOP);
	} else {
		motor_lock(HIGH_RACKET_MOTOR);
	}
}

s32 racket_current_time(void) { return current_time; }
u32 high_racket_move_time(void) { return request_high_racket_move_time; }
s32 get_low_speed(void) { return low_racket_speed; }
s32 get_high_speed(void) { return high_racket_speed; }
bool get_low_switch(void) { return low_switch; }
bool get_high_switch(void) { return high_switch; }
u8 get_low_mode(void) { return low_racket_mode; }
u8 get_high_mode(void) { return high_racket_mode; }
