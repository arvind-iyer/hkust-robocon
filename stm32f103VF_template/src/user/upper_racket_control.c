#include "upper_racket_control.h"
#include "delay.h"

static u32 current_time = 0;
static bool s1_switch = 0;
static bool s2_switch = 0;
static bool b1_switch = 0;
static bool b2_switch = 0;
static bool b3_switch = 0;

static s32 pos_s1_encoder_value = 0;	// MOTOR7
static s32 pos_s2_encoder_value = 0;	// MOTOR7
static s32 pos_b1_encoder_value = 0;	// MOTOR6
static s32 pos_b2_encoder_value = 0;	// MOTOR6
static s32 pos_b3_encoder_value = 0;	// MOTOR5
static s32 target_motor5_encoder_value = 0;
static s32 target_motor6_encoder_value = 0;
static s32 target_motor7_encoder_value = 0;

static bool low_calibrate_mode = 0;
static bool low_calibrated = 0;
static u8 low_calibrate_switch_off_count = 0;
static bool low_calibrate_leave_switch = 1;
static bool low_racket_moving = 0;
static u32 low_racket_moving_start_time = 0;

static bool high_calibrate_mode = 0;
static bool high_calibrated = 0;
static bool high_racket_moving = 0;
static u32 high_racket_moving_start_time = 0;
static u8 high_racket_status = 0;

static u32 last_rotate_left_time = 0;
static u32 last_rotate_right_time = 0;

static s32 low_speed = 1600;
static s32 high_speed = 1200;
static s32 pivot_speed = 600;

void pivot_rotate_left();
void pivot_rotate_right();

void increase_low_speed() {
	if(low_speed<1799) low_speed += 1;
}
void decrease_low_speed() {
	if(low_speed>0) low_speed -= 1;
}
void increase_high_speed() {
	if(high_speed<1799) high_speed += 1;
}
void decrease_high_speed() {
	if(high_speed>0) high_speed -= 1;
}
void increase_pivot_speed() {
	if(pivot_speed<1799) pivot_speed += 1;
}
void decrease_pivot_speed() {
	if(pivot_speed>0)	pivot_speed -= 1;
}

s32 get_low_speed() { return low_speed; }
s32 get_high_speed() { return high_speed; }
s32 get_pivot_speed() { return pivot_speed; }

u8 get_high_racket_status() { return high_racket_status; }

bool get_s1(void) { return s1_switch; }
bool get_s2(void) { return s2_switch; }
bool get_b1(void) { return b1_switch; }
bool get_b2(void) { return b2_switch; }
bool get_b3(void) { return b3_switch; }
s32 get_b1e() {
	return pos_b1_encoder_value;
}
s32 get_b2e() {
	return pos_b2_encoder_value;
}

void low_racket_move(void) {
	low_racket_moving = 1;
	low_racket_moving_start_time = get_full_ticks();
}

void low_racket_calibrate(void) {
	low_calibrate_mode = 1;
	low_calibrate_switch_off_count = 0;
}

void high_racket_move(void) {
	high_racket_moving = 1;
	high_racket_status = 0;
	high_racket_moving_start_time = get_full_ticks();
}

void high_racket_calibrate(void) {
	high_calibrate_mode = 1;
}

void racket_init(void) {
  register_special_char_function('/', low_racket_move);
	register_special_char_function(';', low_racket_calibrate);
	
	register_special_char_function(' ', high_racket_move);
	register_special_char_function('l', high_racket_calibrate);
	
	register_special_char_function('>', increase_high_speed);
	register_special_char_function('<', decrease_high_speed);
	register_special_char_function('.', increase_low_speed);
	register_special_char_function(',', decrease_low_speed);
	
	register_special_char_function('r', pivot_rotate_left);
	register_special_char_function(']', pivot_rotate_right);
	register_special_char_function('=', increase_pivot_speed);
	register_special_char_function('-', decrease_pivot_speed);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	/* GPIO configuration */
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// Swithces initialization
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = S1_Pin | S2_Pin;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = B1_Pin | B2_Pin | B3_Pin;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void racket_update(void) {
	// !! When S1 or S2 is pressed, the value is 0.
	s1_switch = !GPIO_ReadInputDataBit(GPIOE, S1_Pin);
	s2_switch = !GPIO_ReadInputDataBit(GPIOE, S2_Pin);
	// !! When B1, B2 or B3 are pressed, the value is 1.
	b1_switch =  GPIO_ReadInputDataBit(GPIOE, B1_Pin);
	b2_switch =  GPIO_ReadInputDataBit(GPIOE, B2_Pin);
	b3_switch =  GPIO_ReadInputDataBit(GPIOE, B3_Pin);
	
	current_time =  get_full_ticks();
	
	// "Low racket"
	// Calibrate Mode
	if (low_calibrate_mode == 1) {
		if (b3_switch == 0) {
			motor_set_vel(MOTOR5, 240, OPEN_LOOP);
			low_calibrate_leave_switch = 1;
		} else {
			if(low_calibrate_leave_switch == 1) {
				low_calibrate_switch_off_count += 1;
				low_calibrate_leave_switch = 0;
			}
			if (low_calibrate_switch_off_count >= 2) {
				motor_lock(MOTOR5);
				low_calibrate_mode = 0;
				low_calibrated = 1;
			}
		}
	}
	// Regular Mode
	else if(low_calibrated == 1) {
		if (b3_switch == 1 && (current_time - low_racket_moving_start_time > 100) ) {
			low_racket_moving = 0;
		}
		
		if (low_racket_moving == 1) {
			motor_set_vel(MOTOR5, low_speed, OPEN_LOOP);
		} else {
			motor_lock(MOTOR5);
		}
	}
	
	
	
	// "High racket"
	if (high_calibrate_mode == 1) {
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
		if (high_racket_moving == 1) {
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
	}
	
	// Pivot
	if (!s1_switch && last_rotate_left_time!=0 && (current_time - last_rotate_left_time < 15) ) {
		motor_set_vel(MOTOR7, -pivot_speed, OPEN_LOOP);
	} else if (!s2_switch && last_rotate_right_time!=0 && (current_time - last_rotate_right_time < 15) ) {
		motor_set_vel(MOTOR7, pivot_speed, OPEN_LOOP);
	} else {
		motor_lock(MOTOR7);
	}
		
}
	
	// Pivot
void pivot_rotate_left(void){
	last_rotate_left_time = get_full_ticks();
}
void pivot_rotate_right(void){
	last_rotate_right_time = get_full_ticks();
}
	
	
