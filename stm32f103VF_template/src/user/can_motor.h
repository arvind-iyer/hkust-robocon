#include "can.h"
#include "can_protocol.h"

#define CAN_MOTOR_COUNT										16
#define	CAN_MOTOR_BASE										0x0B0

/*** TX ***/
#define	CAN_MOTOR_VEL_LENGTH							6
#define CAN_MOTOR_VEL_CMD									0xAA

/*** RX ***/
#define CAN_ENCODER_FEEDBACK_LENGTH				5
#define CAN_ENCODER_FEEDBACK							0x22


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

