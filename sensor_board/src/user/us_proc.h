#ifndef	__US_PROC_H
#define	__US_PROC_H

#include "stm32f10x.h"
#include "can_protocol.h"
#include "ultrasonic.h"

#define	US_CAN_PROC_CMD		0x10

typedef struct {
	u32 in_range_full_ticks, out_range_full_ticks;
	u16 in_distance; 
	u16 in_range_time;
} US_PROC_TypeDef;


u8 us_in_range(u8 i);
void us_proc_init(void);
void us_proc_update(void);
u16 us_proc_in_range_time(u8 i);


#endif	/* __US_PROC_H */
