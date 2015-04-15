#include "racket_control.h"
#include "buzzer_song.h"
#include "delay.h"

static u32 current_time = 0;
static bool stop_switch = 0;
static bool low_switch = 0;
static bool high_switch = 0;
static bool stop_flag = 0;

// Lower racket variables
static u8   low_racket_mode = 0;
static s32  low_racket_speed = 240;
static bool request_low_racket_move = 0;
static u32  request_low_racket_move_time = 0;
static bool use_high_switch = false;

void low_racket_move(void) {
	request_low_racket_move = 1;
	request_low_racket_move_time = get_full_ticks();
}

void low_racket_stop() {
	request_low_racket_move = 0;
}

void sensor_init(void) {
}

void racket_init(void) {
	/* GPIO configuration */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = Switch_Low_Pin | Switch_High_Pin;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void sensor_update(void) {
	
}

void racket_update(void) {
	// When B1 and B2 are pressed, their values are 1.
	if (use_high_switch == true)
		stop_switch  = GPIO_ReadInputDataBit(GPIOE, Switch_High_Pin);
	else
		stop_switch = GPIO_ReadInputDataBit(GPIOE, Switch_High_Pin); // Low
	
	if (stop_switch == true) {
		stop_flag = 1;
	}
	
	low_racket_update();
}


/**
	* @brief Move or stop the lower racket according to other variables. ( Called by racket_update() )
  */
void low_racket_update() {
	current_time = get_full_ticks();
	
	if (low_racket_mode == 0) {
		low_racket_speed = 240;
		if (stop_switch == 1 && (current_time - request_low_racket_move_time > 500) ) {
			low_racket_mode = 1;
			request_low_racket_move = 0;
			stop_flag = 0;
		}
	} else if (low_racket_mode == 1) {
		low_racket_speed = LOW_RACKET_HIT_SPEED;
		if (stop_switch == 1 && (current_time - request_low_racket_move_time > 50) ) {
			request_low_racket_move = 0;
			stop_flag = 0;
		}
	} else {
		low_racket_speed = 240;
		request_low_racket_move = 0;
		stop_flag = 0;
	}
	
	if(request_low_racket_move == 1) {
		motor_set_vel(LOW_RACKET_MOTOR, low_racket_speed, OPEN_LOOP);
	} else {
		motor_lock(LOW_RACKET_MOTOR);
	}
}

s32 racket_current_time(void) { return current_time; }
u32 low_racket_move_time(void) { return request_low_racket_move_time; }
s32 get_low_speed(void) { return low_racket_speed; }
bool get_low_switch(void) { return low_switch; }
bool get_high_switch(void) { return high_switch; }
u8 get_low_mode(void) { return low_racket_mode; }

void racket_out(void) {
	use_high_switch = 1;
}
void racket_in(void) {
	use_high_switch = 0;
}
