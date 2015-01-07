#include "can.h"
#include "can_protocol.h"

typedef enum {
	MOTOR1 = 0x0B0,
	MOTOR2 = 0x0B1,
	MOTOR3 = 0x0B2,
	MOTOR4 = 0x0B3,
	MOTOR5 = 0x0B4,
	MOTOR6 = 0x0B5,
	MOTOR7 = 0x0B6,
	MOTOR8 = 0x0B7,
	MOTOR9 = 0x0B8,
	MOTOR10 = 0x0B9,
	MOTOR11 = 0x0BA,
	MOTOR12 = 0x0BB,
	MOTOR13 = 0x0BC,
	MOTOR14 = 0x0BD,
	MOTOR15 = 0x0BE,
	MOTOR16 = 0x0BF
} MOTOR_ID;


typedef enum {
	OPEN_LOOP = 0,
	CLOSE_LOOP = 1
} CLOSE_LOOP_FLAG;

