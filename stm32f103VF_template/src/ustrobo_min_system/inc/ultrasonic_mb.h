#ifndef	__ULTRASONIC_MB_H
#define	__ULTRASONIC_MB_H

#include "stm32f10x.h"
#include "can_protocol.h"

#define	US_CAN_ID						0x200
#define	US_DEVICE_COUNT			15


#define	US_CAN_DISTANCE_CMD				0x00
#define	US_CAN_DISTANCE_LENGTH		3

#define	US_CAN_PROC_CMD						0x10
#define	US_CAN_PROC_LENGTH				5

#define	US_CAN_RX_TIMEOUT					250		// 250 ms
typedef struct {
	u16 distance;
} US_MB_Typedef;

void us_mb_init(void);
u16 us_get_distance(u8 id);


#endif	/* __ULTRASONIC_MB_H */
