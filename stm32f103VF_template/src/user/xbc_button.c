#include "xbc_button.h"
#include "xbc_mb.h"
#include "button.h"
#include "racket_control.h"
#include "wheel_base.h"
#include "wheel_base_pid.h"
#include "buzzer.h"

const int SPEED_CHANGE_TIMEOUT = 150;
const int PNEUMATIC_CHANGE_TIMEOUT = 300;

static u32 last_decreased_speed = 0;
static u32 last_increased_speed = 0;
static u32 last_toggled_pneumatic = 0;

static u32 xbc_last_received_nonzero_speed_timer = 0;
static u32 xbc_last_received_nonzero_angle_timer = 0;
// handler for all xbox buttons

void xbc_button_handler(void)
{
	if (xbc_get_joy(XBC_JOY_LX) || xbc_get_joy(XBC_JOY_LY) || xbc_get_joy(XBC_JOY_LT) || xbc_get_joy(XBC_JOY_RT))
	{
		xbc_last_received_nonzero_speed_timer = get_full_ticks();
		if (xbc_get_joy(XBC_JOY_LT) || xbc_get_joy(XBC_JOY_RT)) {
			xbc_last_received_nonzero_angle_timer = get_full_ticks();
		}
	}
	
	if (button_pressed(BUTTON_XBC_B))
	{
		upper_hit();
	}
	
	if (button_pressed(BUTTON_XBC_Y))
	{
		static bool pneumatic_stat = 0;
		if ((get_full_ticks() - last_toggled_pneumatic) > PNEUMATIC_CHANGE_TIMEOUT) {
			if (pneumatic_stat) {
				close_pneumatic();
			} else {
				open_pneumatic();
			}
			pneumatic_stat = !pneumatic_stat;
			last_toggled_pneumatic = get_full_ticks();
		}
	}
	
	if (button_pressed(BUTTON_XBC_A))
	{
		serving();
	}
	
	if (button_pressed(BUTTON_XBC_X))
	{
		racket_calibrate();
	}
	
	if (button_pressed(BUTTON_XBC_N))
	{
		set_starting_pos();
	}
	
	if (button_pressed(BUTTON_XBC_S))
	{
		set_after_serve_pos();
	}
	
	if (button_pressed(BUTTON_XBC_E))
	{
		set_serving_pos();
	}
	
	if (button_pressed(BUTTON_XBC_W))
	{
		set_returning_pos();
	}
	
	if (button_pressed(BUTTON_XBC_LB))
	{
		wheel_base_pid_on();
	}
	
	if (button_pressed(BUTTON_XBC_RB))
	{
		wheel_base_pid_off();
	}
	
	if (button_pressed(BUTTON_XBC_START))
	{
		enable_ultrasonic_sensor();
	}
	
	if (button_pressed(BUTTON_XBC_BACK))
	{
		disable_ultrasonic_sensor();
	}
	
	if (xbc_get_joy(XBC_JOY_RX)) {
		if (xbc_get_joy(XBC_JOY_RX) > 0 && (get_full_ticks() - last_increased_speed) > SPEED_CHANGE_TIMEOUT) {
			u8 speed_mode = wheel_base_get_speed_mode();
			if (speed_mode < 9) {
				++speed_mode;
			}
			MUSIC_NOTE speed_changed_sound[] = {{(MUSIC_NOTE_LETTER)(speed_mode + 1), 6}, NOTE_END};
			buzzer_play_song(speed_changed_sound, 50, 0);
			wheel_base_set_speed_mode(speed_mode);
			last_increased_speed = get_full_ticks();
		} else if (xbc_get_joy(XBC_JOY_RX) < 0 && (get_full_ticks() - last_decreased_speed) > SPEED_CHANGE_TIMEOUT) {
			u8 speed_mode = wheel_base_get_speed_mode();
			if (speed_mode > 0)
			{
				--speed_mode;
			}
			MUSIC_NOTE speed_changed_sound[] = {{(MUSIC_NOTE_LETTER)(speed_mode + 1), 6}, NOTE_END};
			buzzer_play_song(speed_changed_sound, 50, 0);
			wheel_base_set_speed_mode(speed_mode);
			last_decreased_speed = get_full_ticks();
		}
	}		
}

// function to get last non-zero speed received

u32 xbc_get_received_nonzero_speed_timer(void)
{
	return xbc_last_received_nonzero_speed_timer;
}

u32 xbc_get_received_nonzero_angle_timer(void)
{
	return xbc_last_received_nonzero_angle_timer;
}