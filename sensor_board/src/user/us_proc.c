#include "us_proc.h"
#include "ticks.h"

static US_PROC_TypeDef us_proc[US_DEVICE_COUNT];


u8 us_in_range(u8 i) 
{
	return us_get_distance(i) > 20 && us_get_distance(i) <= 1600;
}

static void us_can_proc_tx(u8 id, u16 in_range_time, u16 in_distance)
{
	CAN_MESSAGE msg;
	msg.id = (US_CAN_ID + id);
	msg.length = 5;

	msg.data[1] = (u8) ((in_range_time >> 8) & 0xFF);
	msg.data[2] = (u8) (in_range_time & 0xFF);
	msg.data[3] = (u8) ((in_distance >> 8) & 0xFF);
	msg.data[4] = (u8) (in_distance & 0xFF);
	
	can_tx_enqueue(msg);
}


void us_proc_init(void)
{
	for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
		us_proc[i].in_range_full_ticks = 0;
		us_proc[i].out_range_full_ticks = 0;
		us_proc[i].in_distance = 0;
		us_proc[i].in_range_time = 0;
	}
}

void us_proc_update(void)
{
	for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
		if (us_proc[i].in_range_full_ticks == 0 && us_in_range(i)) {
			// Trigger
			us_proc[i].in_range_full_ticks = get_full_ticks();
			us_proc[i].in_distance = us_get_distance(i);
		} else if (us_proc[i].in_range_full_ticks != 0 && !us_in_range(i)) {
			// Falling
			us_proc[i].out_range_full_ticks = get_full_ticks(); 
			us_proc[i].in_range_time = us_proc[i].out_range_full_ticks - us_proc[i].in_range_full_ticks;
			
			us_proc[i].in_range_full_ticks = 0;
			
			us_can_proc_tx(i, us_proc[i].in_range_time, us_proc[i].in_distance);
		}
	}
}

u16 us_proc_in_range_time(u8 i)
{
	return us_proc[i].in_range_time;
}




