#include "button_event.h"

bool speed_button_released_before = false;
bool home_pressed_before = false;

void button_event_wheel_base_set_vel(s32 x, s32 y, s32 w) {
	wheel_base_set_vel(x, y, w);
	wheel_base_vel_last_update_refresh();
}

void button_event_update(void)
{
	// Wheel Base
	u16 speed_ratio = SPEED_MODES[wheel_base_get_speed_mode()];
	s32 axis_speed = ANALOG_SPEED * speed_ratio / 100;
	s32 diagonal_speed = ANALOG_SPEED * 707 / 1000 * speed_ratio / 100;
	s32 angle_speed;
	
	if ( button_pressed(BUTTON_XBC_LB) ) {
		angle_speed = -axis_speed;
	} else if (button_pressed(BUTTON_XBC_RB) ) {
		angle_speed = axis_speed;
	} else {
		angle_speed = 0;
	}
	
	if ( xbc_get_joy(XBC_JOY_LX) != 0 || xbc_get_joy(XBC_JOY_LY) != 0 ) {
		button_event_wheel_base_set_vel(
			axis_speed * xbc_get_joy(XBC_JOY_LX) / 1000,	// x
			axis_speed * xbc_get_joy(XBC_JOY_LY) / 1000,	// y
			angle_speed);
	} else if ( button_pressed(BUTTON_XBC_N) ) {
		button_event_wheel_base_set_vel(0, axis_speed, angle_speed);
	} else if ( button_pressed(BUTTON_XBC_E) ) {
		button_event_wheel_base_set_vel(axis_speed, 0, angle_speed);
	} else if ( button_pressed(BUTTON_XBC_S) ) {
		button_event_wheel_base_set_vel(0, -axis_speed, angle_speed);
	} else if ( button_pressed(BUTTON_XBC_W) ) {
		button_event_wheel_base_set_vel(-axis_speed, 0, angle_speed);
	} else if ( button_pressed(BUTTON_XBC_NE) ) {
		button_event_wheel_base_set_vel(diagonal_speed, diagonal_speed, angle_speed);
	} else if ( button_pressed(BUTTON_XBC_SE) ) {
		button_event_wheel_base_set_vel(diagonal_speed, -diagonal_speed, angle_speed);
	} else if ( button_pressed(BUTTON_XBC_SW) ) {
		button_event_wheel_base_set_vel(-diagonal_speed, -diagonal_speed, angle_speed);
	} else if ( button_pressed(BUTTON_XBC_NW) ) {
		button_event_wheel_base_set_vel(-diagonal_speed, diagonal_speed, angle_speed);
	} else if ( angle_speed != 0 ) {
		button_event_wheel_base_set_vel(0, 0, angle_speed);
	}
	
	if (button_pressed(BUTTON_XBC_XBOX)) {
		if (home_pressed_before == false) {
			auto_move_mode_switch();
			home_pressed_before = true;
		}
	} else {
		home_pressed_before = false;
	}
	
	if (xbc_get_joy(XBC_JOY_RY)<0 && xbc_get_joy(XBC_JOY_RX)==0) {
		low_racket_standby();
	}
	
	// Speed mode adjustment
	if ( speed_button_released_before && (button_pressed(BUTTON_XBC_START) || button_pressed(BUTTON_XBC_BACK) ) ) {
		u8 speed_mode = wheel_base_get_speed_mode();
		if ( button_pressed(BUTTON_XBC_START) && speed_mode < 9 ) {
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
			default:
				FAIL_MUSIC;
		}
		speed_button_released_before = false;
	} else if( !button_pressed(BUTTON_XBC_START) && !button_pressed(BUTTON_XBC_BACK) ) {
		speed_button_released_before = true;
	}
	
	// Rackets & sensors
	if ( button_pressed(BUTTON_XBC_A) )
		low_racket_move();
	if ( button_pressed(BUTTON_XBC_Y) || button_pressed(BUTTON_XBC_B) || button_pressed(BUTTON_XBC_X) )
		high_racket_move();
	if ( button_pressed(BUTTON_XBC_R_JOY) )
		high_racket_startup();
}
