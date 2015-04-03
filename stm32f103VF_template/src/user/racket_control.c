#include "racket_control.h"
#include "buzzer_song.h"
#include "delay.h"

static u32 current_time = 0;
static bool b1_switch = 0;
static bool b2_switch = 0;
static bool b3_switch = 0;

static s32 pos_b1_encoder_value = 0;	// MOTOR6
static s32 pos_b2_encoder_value = 0;	// MOTOR6
static s32 pos_b3_encoder_value = 0;	// MOTOR5
static s32 target_motor5_encoder_value = 0;
static s32 target_motor6_encoder_value = 0;
static s32 target_motor7_encoder_value = 0;

// Lower racket variables
static u8   low_racket_mode = 0;
static s32  low_racket_speed = 240;
static bool request_low_racket_move = 0;
static u32  request_low_racket_move_time = 0;

// Higher racket variables
static bool high_calibrate_mode = 0;
static bool high_calibrated = 0;
static bool request_high_racket_move = 0;
static u32 request_high_racket_move_time = 0;
static u8 high_racket_status = 0;

static s32 high_speed = 1200;

static bool auto_move_mode_flag = 0;
static bool sensor_sound_on = 1;

void increase_high_speed(void) {
	if(high_speed<1799) high_speed += 1;
}
void decrease_high_speed(void) {
	if(high_speed>0) high_speed -= 1;
}

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
	high_racket_status = 0;
	request_high_racket_move_time = get_full_ticks();
}

void high_racket_startup(void) {
	high_calibrate_mode = 1;
}

void auto_move_mode_on(void) {
	auto_move_mode_flag = 1;
}
void auto_move_mode_off(void) {
	auto_move_mode_flag = 0;
}
void auto_move_mode_switch(void) {
	auto_move_mode_flag = !auto_move_mode_flag;
}

void sensor_sound_switch(void) {
	sensor_sound_on = !sensor_sound_on;
}

void sensor_init(void) {
	register_special_char_function('j', auto_move_mode_on);
	register_special_char_function('k', auto_move_mode_off);
	register_special_char_function('g', sensor_sound_switch);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = S0_Power;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOE, S0_Power, 0);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = S0_Pin;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

}

void racket_init(void) {
	register_special_char_function('/', low_racket_move);
	register_special_char_function('.', low_racket_standby);
	
	register_special_char_function('>', increase_high_speed);
	register_special_char_function('<', decrease_high_speed);
	register_special_char_function('m', high_racket_move);
	register_special_char_function('l', high_racket_startup);
	
	/* GPIO configuration */
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = B1_Pin | B2_Pin | B3_Pin;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void sensor_update(void) {
	//if ( GPIO_ReadInputDataBit(GPIOE, S1_Pin) || GPIO_ReadInputDataBit(GPIOE, S2_Pin) || GPIO_ReadInputDataBit(GPIOE, S3_Pin) )
	if (GPIO_ReadInputDataBit(GPIOE, S0_Pin)==1) {
		if (sensor_sound_on == 1) {
			SUCCESSFUL_MUSIC;
		}
		if(auto_move_mode_flag == 1) {
			if(request_low_racket_move == 0) {
				auto_move_mode_flag = 0;
				request_low_racket_move = 1;
				request_low_racket_move_time = get_full_ticks();
			}
		}
	}
}

void racket_update(void) {
	if(auto_move_mode_flag == 1) {
		CLICK_MUSIC;
	}
	// !! When B1, B2 or B3 are pressed, the value is 1.
	b1_switch = GPIO_ReadInputDataBit(GPIOE, B1_Pin);
	b2_switch = GPIO_ReadInputDataBit(GPIOE, B2_Pin);
	b3_switch = GPIO_ReadInputDataBit(GPIOE, B3_Pin);
	
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
		if (b3_switch == 1 && (current_time - request_low_racket_move_time > 500) ) {
			low_racket_mode = 1;
			request_low_racket_move = 0;
		}
	} else if (low_racket_mode == 1) {
		low_racket_speed = LOW_RACKET_HIT_SPEED;
		if (b3_switch == 1 && (current_time - request_low_racket_move_time > 100) ) {
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
		motor_set_vel(MOTOR5, low_racket_speed, OPEN_LOOP);
	} else {
		motor_lock(MOTOR5);
		
	}
}

/**
	* @brief Move or stop the higher racket according to other variables. ( Called by racket_update() )
  */
void high_racket_update() {
	current_time = get_full_ticks();
	
	if (high_calibrate_mode == 1) {
	// Startup
		if (!pos_b1_encoder_value) {
			if (b1_switch == 0) {
				motor_set_vel(MOTOR6, DIR_TOWARDS_B1 * 300, OPEN_LOOP);
			} else {
				motor_lock(MOTOR6);
				pos_b1_encoder_value = get_encoder_value(MOTOR6);
			}
		} else if (!pos_b2_encoder_value) {
			if(b2_switch==0) {
				motor_set_vel(MOTOR6, DIR_TOWARDS_B2 * 300, OPEN_LOOP);
			} else {
				motor_lock(MOTOR6);
				pos_b2_encoder_value = get_encoder_value(MOTOR6);
			}
		} else if (pos_b1_encoder_value && pos_b2_encoder_value) {
			if(b1_switch == 0) {
				motor_set_vel(MOTOR6, DIR_TOWARDS_B1 * 300, OPEN_LOOP);
			} else {
				//temp = get_encoder_value(MOTOR6);
				motor_lock(MOTOR6);
				high_calibrated = 1;
				high_calibrate_mode = 0;
			}
		}
	} else if (high_calibrated == 1) {
	// After Startup
		if (request_high_racket_move == 1) {
			// When the B2 switch is hit, go back to B1 and finish the command
			if (high_racket_status == 0 && b2_switch==1) {
				high_racket_status = 1;
				motor_lock(MOTOR6);
			} else if (high_racket_status == 1 && b1_switch==1) {
				high_racket_status = 2;
				motor_lock(MOTOR6);
			}
			
			if (high_racket_status == 0) {
				motor_set_vel(MOTOR6, DIR_TOWARDS_B2 * high_speed, OPEN_LOOP);
			} else if (high_racket_status == 1) {
				motor_set_vel(MOTOR6, DIR_TOWARDS_B1 * high_speed, OPEN_LOOP);
			} else if (high_racket_status == 2) {
				motor_lock(MOTOR6);
			}

		}
	} else {
		request_high_racket_move = 0;
	}
}

s32 get_low_racket_speed() { return low_racket_speed; }
s32 get_low_racket_mode() { return low_racket_mode; }
s32 get_high_speed() { return high_speed; }
u8 get_high_racket_status() { return high_racket_status; }
bool get_b1(void) { return b1_switch; }
bool get_b2(void) { return b2_switch; }
bool get_b3(void) { return b3_switch; }
s32 get_b1e() { return pos_b1_encoder_value; }
s32 get_b2e() { return pos_b2_encoder_value; }