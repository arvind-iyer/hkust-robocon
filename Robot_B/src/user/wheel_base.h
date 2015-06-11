#ifndef __WHEEL_BASE_H
#define __WHEEL_BASE_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "can_motor.h"
#include "bluetooth.h"
#include "ticks.h"
#include "gyro.h"
#include "wheel_base_pid.h"

// Protocol
#define	BLUETOOTH_WHEEL_BASE_VEL_ID	    			0x40    // RX
#define	BLUETOOTH_WHEEL_BASE_SPEED_MODE_ID		0x41    // RX
#define	BLUETOOTH_WHEEL_BASE_AUTO_POS_ID			0x50    // RX
#define	BLUETOOTH_WHEEL_BASE_AUTO_START_ID		0x51    // RX
#define	BLUETOOTH_WHEEL_BASE_AUTO_STOP_ID			0x52    // RX
#define BLUETOOTH_WHEEL_BASE_POS_ID						0x60    // TX
#define BLUETOOTH_WHEEL_BASE_CHAR_ID          0x70    // RX


#define	BLUETOOTH_WHEEL_BASE_TIMEOUT					100

#define WHEEL_BASE_REGULAR_CAN_TX							2000

// Wheel-base speed related
#define	WHEEL_BASE_XY_VEL_RATIO								707		//  Last tested value: 1000 (100.0%) with SPEED_MODES stepping value 10
#define	WHEEL_BASE_W_VEL_RATIO								-500	//	Last tested value: -500 (-80.0%)
static const u16 SPEED_MODES[10] =	// In percentage (20 = 20%)
{
	0, 20, 40, 60, 80, 100, 120, 140, 160, 180
};

// Initialized value
#define WHEEL_BASE_DEFAULT_ACC								200
#define	WHEEL_BASE_DEFAULT_SPEED_MODE					1			// from 0 to 9

// Wheel base motors acceleration (CONSTANT, to be configured upon startup)
#define	WHEEL_BASE_BR_ACC											200		// Bottom-right wheel
#define	WHEEL_BASE_BL_ACC										  200		// Bottom-left wheel
#define	WHEEL_BASE_TL_ACC											200		// Top-left wheel
#define	WHEEL_BASE_TR_ACC											200 		// Top-right wheel

typedef struct {
	s32 x;
	s32 y;
	s32 w;		// Angular velocity
} WHEEL_BASE_VEL;

static const MOTOR_ID
	MOTOR_BOTTOM_RIGHT 		= MOTOR3,
	MOTOR_BOTTOM_LEFT 		= MOTOR4,
	MOTOR_TOP_LEFT 				= MOTOR1,
	MOTOR_TOP_RIGHT 			= MOTOR2;


typedef	struct {
	s32 Kp, Ki, Kd;
} PID;

static s32 mvtl, mvtr, mvbl, mvbr;

void wheel_base_init(void);
void wheel_base_set_speed_mode(u8 s);
u8 wheel_base_get_speed_mode(void);
void wheel_base_tx_acc(void);
void wheel_base_set_vel(s32 x, s32 y, s32 w);
WHEEL_BASE_VEL wheel_base_get_vel(void);
WHEEL_BASE_VEL wheel_base_get_vel_prev(void);
s32 wheel_base_get_vel_top_left(void);
s32 wheel_base_get_vel_top_right(void);
s32 wheel_base_get_vel_bottom_left(void);
s32 wheel_base_get_vel_bottom_right(void);
void wheel_base_update(void);
void wheel_base_tx_position(void);


void wheel_base_set_pid(PID pid);
POSITION wheel_base_get_target_pos(void);
void wheel_base_set_target_pos(POSITION pos);
void wheel_base_pid_on(void);
void wheel_base_pid_off(void);
u8 wheel_base_get_pid_flag(void);
void wheel_base_vel_last_update_refresh(void);

// For debugging purpose only
void wheel_base_override_change_speed(void);
void wheel_base_override_set_vel(s32 x, s32 y);


#endif	/* __WHEEL_BASE_H */
