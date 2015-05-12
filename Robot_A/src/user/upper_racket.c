#include "upper_racket.h"

static const GPIO* upper_racket_valve[UPPER_RACKET_VALVE_COUNT] = UPPER_RACKET_VALVES;

static u32 racket_pre_ticks = 0;
static u32 racket_hit_ticks = 0;
static u32 racket_post_ticks = 0;

static u16 racket_pre_delay = 0;

void upper_racket_init(void)
{
	for (u8 i = 0; i < UPPER_RACKET_VALVE_COUNT; ++i) {
		gpio_init(upper_racket_valve[i], GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
		gpio_write(upper_racket_valve[i], (BitAction) 0);
	}
	
	racket_hit_ticks = 0;
	racket_post_ticks = 0;
}

static void upper_racket_set(u8 mode)
{
	for (u8 i = 0; i < UPPER_RACKET_VALVE_COUNT; ++i) {
		gpio_write(upper_racket_valve[i], (BitAction) mode);
	}
}

void upper_racket_e_stop(void)
{
	racket_pre_ticks = 0;
	racket_hit_ticks = 0;
	racket_post_ticks = 0;
	upper_racket_set(0);
}

/**
	* @brief Start hitting the upper racket after pre_delay ms
	* @param pre_delay: Delay (in millisecond)
	* @retval True if the hit starts, false if hit cannot be done due to unfinished hit
	*/
bool upper_racket_hit(u16 pre_delay)
{
	if (racket_pre_ticks == 0 && racket_hit_ticks == 0 && racket_post_ticks == 0) {
			racket_pre_ticks = get_full_ticks();
			racket_pre_delay = pre_delay;
			upper_racket_update(); 
			return true;
	} else {
		buzzer_control_note(2, 100, NOTE_A, 5);
		return false;
	}
	
}


void upper_racket_update(void)
{	
	if (racket_pre_ticks != 0) {
		if (racket_pre_ticks + racket_pre_delay <= get_full_ticks()) {
			upper_racket_set(1);		// Racket hit on
			racket_pre_ticks = 0;
			racket_hit_ticks = get_full_ticks();
		} else {
			buzzer_control_note(2, 100, NOTE_C, 5);
		}
	}
	
	if (racket_hit_ticks != 0) {
		if (racket_hit_ticks + UPPER_RACKET_HIT_DELAY <= get_full_ticks()) {
			upper_racket_set(0);		// Racket hit off
			racket_hit_ticks = 0;
			racket_post_ticks = get_full_ticks();
		} 
	}
	
	if (racket_post_ticks != 0) {
		if (racket_post_ticks + UPPER_RACKET_POST_DELAY <= get_full_ticks()) {
			racket_post_ticks = 0;
		} 
	} 
}

