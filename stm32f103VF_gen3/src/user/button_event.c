#include "button_event.h"

static bool speed_button_released_before = false;
static bool side_control = true;

static u32 home_holding_count = 0;
static u32 start_and_select_holding_count = 0;

static u16 speed_ratio;
static s32 axis_speed;
static s32 diagonal_speed;
static s32 angle_speed;

void button_event_wheel_base_set_vel(s32 x, s32 y, s32 w) {
	wheel_base_set_vel(x, y, w);
	wheel_base_vel_last_update_refresh();
}

void button_event_update(void)
{
	// Wheel Base
	/* u16 */ speed_ratio = SPEED_MODES[wheel_base_get_speed_mode()];
	/* s32 */ axis_speed = ANALOG_SPEED * speed_ratio / 100;
	/* s32 */ diagonal_speed = ANALOG_SPEED * 707 / 1000 * speed_ratio / 100;
	/* s32 angle_speed */;
	
	if (button_pressed(BUTTON_XBC_XBOX)) {
		home_holding_count += 1;
	} else {
		home_holding_count = 0;
	}
		
	if (button_pressed(BUTTON_PS4_SELECT) && button_pressed(BUTTON_PS4_START)) {
		start_and_select_holding_count += 1;
	} else {
		start_and_select_holding_count = 0;
	}
	
	if (xbc_get_joy(XBC_JOY_LT)!=0) {
		angle_speed = -axis_speed * xbc_get_joy(XBC_JOY_LT) / 255;
	} else if (xbc_get_joy(XBC_JOY_RT)!=0) {
		angle_speed = axis_speed * xbc_get_joy(XBC_JOY_RT) / 255;
	} else {
		angle_speed = 0;
	}
	
	if (button_pressed(BUTTON_XBC_LB) && button_pressed(BUTTON_XBC_RB)) {
		button_event_wheel_base_set_vel(0, 0, 0);
	} else if ( xbc_get_joy(XBC_JOY_LX) != 0 || xbc_get_joy(XBC_JOY_LY) != 0 ) {
		if (side_control == true) {
			button_event_wheel_base_set_vel(
				axis_speed * -xbc_get_joy(XBC_JOY_LY) / 1000,	// x
				axis_speed * xbc_get_joy(XBC_JOY_LX) / 1000,	// y
				angle_speed
			);
		} else {
			button_event_wheel_base_set_vel(
				axis_speed * xbc_get_joy(XBC_JOY_LX) / 1000,	// x
				axis_speed * xbc_get_joy(XBC_JOY_LY) / 1000,	// y
				angle_speed
			);
		}
	} else if ( button_pressed(BUTTON_PS4_UP) ) {
		if (side_control == 0) {
			button_event_wheel_base_set_vel(0, axis_speed, angle_speed);
		} else { // 左
			button_event_wheel_base_set_vel(-axis_speed, 0, angle_speed);
		}
	} else if ( button_pressed(BUTTON_PS4_RIGHT) ) {
		if (side_control == 0) {
			button_event_wheel_base_set_vel(axis_speed, 0, angle_speed);
		} else { // 前
			button_event_wheel_base_set_vel(0, axis_speed, angle_speed);
		}
	} else if ( button_pressed(BUTTON_PS4_DOWN) ) {
		if (side_control == 0) {
			button_event_wheel_base_set_vel(0, -axis_speed, angle_speed);
		} else { // 右
			button_event_wheel_base_set_vel(axis_speed, 0, angle_speed);
		}
	} else if ( button_pressed(BUTTON_PS4_LEFT) ) {
		if (side_control == 0) {
			button_event_wheel_base_set_vel(-axis_speed, 0, angle_speed);
		} else { // 後
			button_event_wheel_base_set_vel(0, -axis_speed, angle_speed);
		}
	} else if ( button_pressed(BUTTON_XBC_NE) ) {
		if (side_control == 0) {
			button_event_wheel_base_set_vel(diagonal_speed, diagonal_speed, angle_speed);
		} else { // 左前 (NW)
			button_event_wheel_base_set_vel(-diagonal_speed, diagonal_speed, angle_speed);
		}
	} else if ( button_pressed(BUTTON_XBC_SE) ) {
		if (side_control == 0) {
			button_event_wheel_base_set_vel(diagonal_speed, -diagonal_speed, angle_speed);
		} else { // 右前 (NE)
			button_event_wheel_base_set_vel(diagonal_speed, diagonal_speed, angle_speed);
		}
	} else if ( button_pressed(BUTTON_XBC_SW) ) {
		if (side_control == 0) {
			button_event_wheel_base_set_vel(-diagonal_speed, -diagonal_speed, angle_speed);
		} else { // 右後 (SE)
			button_event_wheel_base_set_vel(diagonal_speed, -diagonal_speed, angle_speed);
		}
	} else if ( button_pressed(BUTTON_XBC_NW) ) {
		if (side_control == 0) {
			button_event_wheel_base_set_vel(-diagonal_speed, diagonal_speed, angle_speed);
		} else { // 左後 (SW)
			button_event_wheel_base_set_vel(-diagonal_speed, -diagonal_speed, angle_speed);
		}
	} else if ( angle_speed != 0 ) {
		button_event_wheel_base_set_vel(0, 0, angle_speed);
	}
	
	if (home_holding_count == 1 || start_and_select_holding_count == 1 ) {
		side_control = !side_control;
		if (side_control) {
			buzzer_play_song(SIDE_CONTROL_ON_SOUND, 120, 0);
		} else {
			buzzer_play_song(SIDE_CONTROL_OFF_SOUND, 120, 0);
		}
	}
	
	// Speed mode adjustment
	if ( speed_button_released_before && (button_pressed(BUTTON_XBC_START) || button_pressed(BUTTON_XBC_BACK) ) ) {
		u8 speed_mode = wheel_base_get_speed_mode();
		if ( button_pressed(BUTTON_XBC_START) && speed_mode < 10 ) {
			speed_mode += 1;
		} else if ( button_pressed(BUTTON_XBC_BACK) && speed_mode > 0 ) {
			speed_mode -= 1;
		}
		wheel_base_set_speed_mode(speed_mode);
		switch(speed_mode) {
			case 0:
				buzzer_play_song(SPEED_0, 200, 0);
				break;
			case 1:
				buzzer_play_song(SPEED_1, 200, 0);
				break;
			case 2:
				buzzer_play_song(SPEED_2, 200, 0);
				break;
			case 3:
				buzzer_play_song(SPEED_3, 200, 0);
				break;
			case 4:
				buzzer_play_song(SPEED_4, 200, 0);
				break;
			case 5:
				buzzer_play_song(SPEED_5, 200, 0);
				break;
			case 6:
				buzzer_play_song(SPEED_6, 200, 0);
				break;
			case 7:
				buzzer_play_song(SPEED_7, 200, 0);
				break;
			case 8:
				buzzer_play_song(SPEED_8, 200, 0);
				break;
			case 9:
				buzzer_play_song(SPEED_9, 200, 0);
				break;
			case 10:
				buzzer_play_song(SPEED_10, 200, 0);
				break;
			default:
				FAIL_MUSIC;
		}
		speed_button_released_before = false;
	} else if( !button_pressed(BUTTON_XBC_START) && !button_pressed(BUTTON_XBC_BACK) ) {
		speed_button_released_before = true;
	}
	
	if (button_pressed(BUTTON_XBC_LB)) {
		sensor_off();
	} else if (button_pressed(BUTTON_XBC_RB)) {
		sensor_on();
	}
	
	if ((button_pressed(BUTTON_XBC_Y) && button_pressed(BUTTON_XBC_B)) == false) {
		if (button_pressed(BUTTON_XBC_Y)) {
			sensor_decrease_delay();
		} else if (button_pressed(BUTTON_XBC_B)) {
			sensor_increase_delay();
		}
	}
	
	// Rackets & sensors
	if (button_pressed(BUTTON_XBC_X)) {
		forehand_daa_la();
	} else {
		forehand_lok_la();
	}
	
	if (button_pressed(BUTTON_XBC_A)) {
		underarm_daa_la();
	} else {
		underarm_lok_la();
	}
	
}
