#include "button_event.h"

void button_event_wheel_base_set_vel(s32 x, s32 y, s32 w) {
	wheel_base_set_vel(x, y, w);
	wheel_base_vel_last_update_refresh();
}

void button_event_update(void)
{
	// Wheel Base (only digital control now)
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
	
	if ( button_pressed(BUTTON_XBC_N) ) {
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
	
	// Rackets & sensors
	if ( button_pressed(BUTTON_XBC_A) )
		low_racket_move();
	if ( button_pressed(BUTTON_XBC_Y) || button_pressed(BUTTON_XBC_B) || button_pressed(BUTTON_XBC_X) )
		high_racket_move();
	if ( button_pressed(BUTTON_XBC_START) )
		high_racket_startup();
}
