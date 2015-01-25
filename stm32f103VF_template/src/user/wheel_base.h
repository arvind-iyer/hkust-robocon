#ifndef __WHEEL_BASE_H
#define __WHEEL_BASE_H

#include "stm32f10x.h"
#include "can_motor.h"
#include "bluetooth.h"
#include "ticks.h"
#include "gyro.h"
#include "wheel_base_pid.h"

// Protocol
#define	BLUETOOTH_WHEEL_BASE_MANUAL_ID				0x40
#define	BLUETOOTH_WHEEL_BASE_SPEED_MODE_ID		0x41

#define BLUETOOTH_WHEEL_BASE_POS_ID						0x60

#define	BLUETOOTH_WHEEL_BASE_TIMEOUT					100

#define WHEEL_BASE_REGULAR_CAN_TX							2000

// Wheel-base speed related
#define	WHEEL_BASE_XY_VEL_RATIO								707			//  70.7%
#define	WHEEL_BASE_W_VEL_RATIO								-800		//	-80.0%
static const u16 SPEED_MODES[10] =	// In percentage (20 = 20%)
{
	0, 10, 15, 25, 35, 55, 65, 85, 95, 115
};

// Initialized value
#define WHEEL_BASE_DEFAULT_ACC								100
#define	WHEEL_BASE_DEFAULT_SPEED_MODE					3			// from 0 to 9

// Wheel base motors acceleration (CONSTANT, to be configured upon startup)
// Unit (velocity per seconds, or 1000 encoder count per seconds square)
#define	WHEEL_BASE_BR_ACC											100		// Bottom-right wheel
#define	WHEEL_BASE_BL_ACC											100		// Bottom-left wheel
#define	WHEEL_BASE_TL_ACC											100		// Top-left wheel
#define	WHEEL_BASE_TR_ACC											100 	// Top-right wheel

typedef struct {
	s32 x;
	s32 y;
	s32 w;		// Angular velocity
} WHEEL_BASE_VEL;

static const MOTOR_ID
	MOTOR_BOTTOM_RIGHT 		= MOTOR1,
	MOTOR_BOTTOM_LEFT 		= MOTOR2,
	MOTOR_TOP_LEFT 				= MOTOR3,
	MOTOR_TOP_RIGHT 			= MOTOR4;


void wheel_base_init(void);
void wheel_base_set_speed_mode(u8 s);
u8 wheel_base_get_speed_mode(void);
void wheel_base_tx_acc(void);
void wheel_base_set_vel(s32 x, s32 y, s32 w);
WHEEL_BASE_VEL wheel_base_get_vel(void);
void wheel_base_update(void);
void wheel_base_tx_position(void);


#endif	/* __WHEEL_BASE_H */
