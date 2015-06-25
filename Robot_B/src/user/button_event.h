#ifndef __BUTTON_HANDLER_H
#define __BUTTON_HANDLER_H

#include "button.h"
#include "buzzer_song.h"
#include "racket_control.h"
#include "wheel_base.h"
#include "adc_app.h"

#define ANALOG_SPEED 100

#define SIDE_NORMAL 0
#define SIDE_RIGHT  1
#define SIDE_LEFT   2

#define BUTTON_PS4_L1				BUTTON_XBC_LB
#define PS4_JOY_L2					XBC_JOY_LT
#define BUTTON_PS4_R1				BUTTON_XBC_RB
#define PS4_JOY_R2					XBC_JOY_RT
#define BUTTON_PS4_SHARE		BUTTON_XBC_BACK	
#define BUTTON_PS4_OPTIONS	BUTTON_XBC_START
#define BUTTON_PS4_N				BUTTON_XBC_N
#define BUTTON_PS4_E				BUTTON_XBC_E
#define BUTTON_PS4_S				BUTTON_XBC_S
#define BUTTON_PS4_W				BUTTON_XBC_W
#define BUTTON_PS4_NE				BUTTON_XBC_NE
#define	BUTTON_PS4_SE				BUTTON_XBC_SE
#define	BUTTON_PS4_SW				BUTTON_XBC_SW
#define BUTTON_PS4_NW				BUTTON_XBC_NW
#define BUTTON_PS4_SQUARE		BUTTON_XBC_X
#define BUTTON_PS4_CROSS		BUTTON_XBC_A
#define BUTTON_PS4_CIRCLE		BUTTON_XBC_B
#define BUTTON_PS4_TRIANGLE	BUTTON_XBC_Y
#define BUTTON_PS4_PS				BUTTON_XBC_XBOX
#define	BUTTON_PS4_L3				BUTTON_XBC_L_JOY
#define	BUTTON_PS4_R3				BUTTON_XBC_R_JOY
#define PS4_JOY_LX					XBC_JOY_LX
#define PS4_JOY_LY					XBC_JOY_LY
#define PS4_JOY_RX					XBC_JOY_RX
#define PS4_JOY_RY					XBC_JOY_RY
// Buttons Redirect
#define BUTTON_PS4_UP				BUTTON_XBC_N
#define BUTTON_PS4_DOWN			BUTTON_XBC_S
#define BUTTON_PS4_LEFT			BUTTON_XBC_W
#define BUTTON_PS4_RIGHT		BUTTON_XBC_E
#define BUTTON_PS4_START		BUTTON_XBC_START
#define BUTTON_PS4_SELECT		BUTTON_XBC_BACK
#define	BUTTON_PS4_HOME			BUTTON_XBC_XBOX

void gamepad_led_init(void);
void button_event_update(void);
u8 button_event_get_side_control(void);

bool is_force_terminate(void);
s32 get_brake_velocity(void);
u32 get_brake_position(void);

#endif
