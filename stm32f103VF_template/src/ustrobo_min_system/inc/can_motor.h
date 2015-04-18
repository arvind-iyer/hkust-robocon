#ifndef __CAN_MOTOR_H
#define __CAN_MOTOR_H

#include "can_protocol.h"

#define CAN_MOTOR_COUNT								16
#define	CAN_MOTOR_BASE								0x0B0

/*** TX ***/
#define	CAN_MOTOR_VEL_LENGTH					6
#define CAN_MOTOR_VEL_CMD							0xAA

#define CAN_MOTOR_POS_LENGTH 					7
#define CAN_MOTOR_POS_CMD 						0xBB

#define CAN_MOTOR_PARAMETER_LENGTH		3
#define CAN_MOTOR_PARAMETER_CMD				0x44

#define CAN_MOTOR_LOCK_LENGTH	 		  	1
#define CAN_MOTOR_LOCK_CMD						0xEE	

/*** RX ***/
#define CAN_ENCODER_FEEDBACK_LENGTH		5
#define CAN_ENCODER_FEEDBACK					0x22

/***  SENSOR  ***/

#define LIGHT_SENSOR_0 		0x13
#define LIGHT_SENSOR_1 		0x12
#define LIGHT_SENSOR_2		0x14
#define LIGHT_SENSOR_3		0x15
#define LIGHT_SENSOR_4		0x16
#define LIGHT_SENSOR_5		0x17
#define LIGHT_SENSOR_6		0x18
#define LIGHT_SENSOR_7		0x19
#define LIGHT_SENSOR_8		0x1A
#define LIGHT_SENSOR_9		0x1B
#define LIGHT_SENSOR_10		0x1C
#define LIGHT_SENSOR_11		0x1D
#define LIGHT_SENSOR_12		0x1E
#define LIGHT_SENSOR_13		0x1F
#define LIGHT_SENSOR_14		0x20
#define LIGHT_SENSOR_15		0x21
#define COLOR_1 0x31
#define COLOR_2 0x32
#define COLOR_3 0x33






typedef enum {
	MOTOR1 = 0,
	MOTOR2,
	MOTOR3,
	MOTOR4,
	MOTOR5,
	MOTOR6,
	MOTOR7,
	MOTOR8,
	MOTOR9,
	MOTOR10,
	MOTOR11,
	MOTOR12,
	MOTOR13,
	MOTOR14,
	MOTOR15,
	MOTOR16
} MOTOR_ID;

typedef enum {
	OPEN_LOOP = 0,
	CLOSE_LOOP = 1
} CLOSE_LOOP_FLAG;


/*** TX ***/
void can_motor_init(void);
void motor_set_vel(MOTOR_ID motor_id, s32 vel, CLOSE_LOOP_FLAG close_loop_flag);
void motor_set_acceleration(MOTOR_ID motor_id, u16 accel);
void motor_lock(MOTOR_ID motor_id);
static void can_sensor_feedback_decoding(CanRxMsg msg);
void can_sensor_init(void);

u16 get_feedback (u8 i, u8 j);
char get_color (u8 i);


/*** RX ***/
s32 get_encoder_value(MOTOR_ID motor_id);

#endif			// __CAN_MOTOR_H
