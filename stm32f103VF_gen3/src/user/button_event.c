#include "button_event.h"

static u8   side_control = SIDE_RIGHT;

static u32 home_holding_count = 0;
static u32 start_and_select_holding_count = 0;
static u32 select_and_l1_holding_count = 0;
static u32 l1_and_cross_holding_count = 0;
static bool speed_button_released_before = false;
static bool underarm_reverse = false;

static u16 speed_ratio;
static s32 axis_speed;
static s32 diagonal_speed;
static s32 angle_speed;
static s32 x_speed;
static s32 y_speed;
static s32 temp_s32;
static u32 l_analog_magnitude;

// This "trigger" means LT (L2 for PS4) & RT (R2 for PS4)
s32 button_event_trigger_value_conversion(s16 trigger_value) {
    if (trigger_value == 255 || trigger_value == -255) {
		return trigger_value;
	} else if (trigger_value > 0) {
		return (trigger_value * trigger_value + 147) / 294 + (trigger_value * 3 + 12) / 23;
	} else {
		return (- trigger_value * trigger_value - 147) / 294 + (trigger_value * 3 - 12) / 23;
	}
}

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
	
	if (button_pressed(BUTTON_PS4_SELECT) && button_pressed(BUTTON_PS4_L1)) {
		select_and_l1_holding_count += 1;
	} else {
		select_and_l1_holding_count = 0;
	}
	
	if (button_pressed(BUTTON_PS4_L1) && button_pressed(BUTTON_PS4_CROSS)) {
		l1_and_cross_holding_count += 1;
	} else {
		l1_and_cross_holding_count = 0;
	}
	
	angle_speed = axis_speed * button_event_trigger_value_conversion(xbc_get_joy(PS4_JOY_R2) - xbc_get_joy(PS4_JOY_L2)) / 255;
	
	x_speed = 0;
	y_speed = 0;
	
	l_analog_magnitude = Sqrt(Sqr(xbc_get_joy(PS4_JOY_LX)) + Sqr(xbc_get_joy(PS4_JOY_LY)));
	
	if (button_pressed(BUTTON_PS4_CIRCLE)) {
		// 減速至零
		button_event_wheel_base_set_vel(0, 0, 0);
	} else {
		if ( xbc_get_joy(PS4_JOY_LX) != 0 || xbc_get_joy(PS4_JOY_LY) != 0 ) {
			x_speed = axis_speed * xbc_get_joy(PS4_JOY_LX) / 1000;
			y_speed = axis_speed * xbc_get_joy(PS4_JOY_LY) / 1000;
		} else if (button_pressed(BUTTON_PS4_UP)) {
			// Forward 前
			// x_speed = 0;
			y_speed = axis_speed;
		} else if (button_pressed(BUTTON_PS4_DOWN)) {
			// Backward 後
			// x_speed = 0;
			y_speed = -axis_speed;
		} else if (button_pressed(BUTTON_PS4_LEFT)) {
			// Left 左
			x_speed = -axis_speed;
			// y_speed = 0;
		} else if (button_pressed(BUTTON_PS4_RIGHT)) {
			// Right 右
			x_speed = axis_speed;
			// y_speed = 0;
		} else if (button_pressed(BUTTON_PS4_NW)) {
			// Left-Forward 左前
			x_speed = -diagonal_speed;
			y_speed = diagonal_speed;
		} else if (button_pressed(BUTTON_PS4_SW)) {
			// Left-Backward 左後
			x_speed = -diagonal_speed;
			y_speed = -diagonal_speed;
		} else if (button_pressed(BUTTON_PS4_NE)) {
			// Right-Forward 右前
			x_speed = diagonal_speed;
			y_speed = diagonal_speed;
		} else if (button_pressed(BUTTON_PS4_SE)) {
			// Right-Backward 右後
			x_speed = diagonal_speed;
			y_speed = -diagonal_speed;
		}
		
		temp_s32 = y_speed;
		if (side_control == SIDE_RIGHT) {
			y_speed = x_speed;
			x_speed = -temp_s32;
		} else if (side_control == SIDE_LEFT) {
			y_speed = -x_speed;
			x_speed = temp_s32;
		}
		
		button_event_wheel_base_set_vel(x_speed, y_speed, angle_speed);
	}
	
	if (home_holding_count == 1 || start_and_select_holding_count == 1 ) {
		if (side_control == SIDE_NORMAL) {
			side_control = SIDE_RIGHT;
			buzzer_play_song(SIDE_CONTROL_ON_SOUND, 120, 0);
		} else {
			side_control = SIDE_NORMAL;
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
	
	//sensor_off();
	//sensor_on();
	if (select_and_l1_holding_count == 1) {
		side_control = SIDE_LEFT;
		buzzer_play_song(SIDE_CONTROL_LEFT_SOUND, 120, 0);
	}
	
	if ((button_pressed(BUTTON_XBC_Y) && button_pressed(BUTTON_XBC_B)) == false) {
		if (button_pressed(BUTTON_XBC_Y)) {
			sensor_decrease_delay();
		} else if (button_pressed(BUTTON_XBC_B)) {
			sensor_increase_delay();
		}
	}
	
	// Rackets & sensors
	
	if (l1_and_cross_holding_count == 1) {
		underarm_reverse = !underarm_reverse;
	}
	
	if (button_pressed(BUTTON_XBC_X)) {
		forehand_daa_la();
	} else {
		forehand_lok_la();
	}
	
	if (button_pressed(BUTTON_XBC_A)) {
		if (underarm_reverse == false)
			underarm_daa_la();
		else
			underarm_lok_la();
	} else {
		if (underarm_reverse == false)
			underarm_lok_la();
		else
			underarm_daa_la();
	}
	
}

u8 button_event_get_side_control(void) {
	return side_control;
}

u32 button_event_get_l_analog_magnitude(void) {
	return l_analog_magnitude;
}
