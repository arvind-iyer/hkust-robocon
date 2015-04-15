#include "racket.h"





// flags
static bool is_pneu_extended = 1; 			// true if pneumatic piston is extended
static u32 racket_pneu_start_time = 0;
static bool racket_delayed_hit_queued=0;
static u32 racket_delayed_hit_start_time=0;


static bool is_pneu_2_extended = 1;		// FOR ROBOT C
static u32 racket_pneu_2_start_time = 0;



/********************** here all getters and setters ends **************************/



void racket_pneumatic_set(bool data)		// 
{
	gpio_write(&PD8, data);
}


void racket_pneumatic_2_set(bool data)	// for robot C
{
	gpio_write(PNEU_GPIO_DOWN, data);
	//log("pneu2",is_pneu_2_extended);
}

//Called ever 10ms to check and update racket data and redirect to starting and stopping racket at required points
void racket_update(void)
{
	if (ROBOT=='D')
		serve_update();
	
	if (racket_delayed_hit_queued && get_full_ticks() > racket_delayed_hit_start_time + RACKET_HIT_DELAY_TIME)
	{
		racket_delayed_hit_queued=0;
		racket_hit();
		log("rac delayed hit",0);
	}
	
	if(is_pneu_extended && (get_full_ticks() > racket_pneu_start_time + 500/*RACKET_SERVE_DELAY*/))
	{
		is_pneu_extended = 0;
		racket_pneumatic_set(is_pneu_extended);
	}
	if (ROBOT=='C' && is_pneu_2_extended && (get_full_ticks() > racket_pneu_2_start_time + 500))
	{
		is_pneu_2_extended = 0;
		racket_pneumatic_2_set(is_pneu_2_extended);
	}
	
	if (!gpio_read_input(LASER_GPIO))
	{
		//racket_hit();
	}
	
	/*if (ROBOT=='C' && !gpio_read_input(LASER_GPIO) && racket_laser_not_alligned)
	{
		racket_laser_not_alligned=0;
		racket_hit();
		serve_enabled = 0;
		racket_last_laser_trigger_time=get_full_ticks();

	}*/
	/*
	if (ROBOT=='C' && !racket_laser_not_alligned && gpio_read_input(LASER_GPIO) && get_full_ticks() > racket_last_laser_trigger_time + racket_laser_trigger_interval)		//trigger racket with laser
	{
		////serve_enabled = 0;
		
		serve_enabled = 0;
		racket_last_laser_trigger_time=get_full_ticks();
		racket_hit();
		racket_laser_not_alligned=1;
		
		
		////racket_laser_trigger_enabled=0;
	}*/
	
}

void racket_delayed_hit(int delay)
{
	if(!is_pneu_extended && !racket_delayed_hit_queued)
	{
		racket_delayed_hit_queued=1;
		racket_delayed_hit_start_time=get_full_ticks();
		RACKET_HIT_DELAY_TIME = delay;
	}
	
	
}

void racket_hit(void)
{
	if (!is_pneu_extended)
	{
		is_pneu_extended = 1;
		racket_pneumatic_set(is_pneu_extended);
		racket_pneu_start_time = get_full_ticks();
		
	}
	
}
void racket_down_hit(void)
{
	if (!is_pneu_2_extended)
	{
		is_pneu_2_extended = 1;
		racket_pneumatic_2_set(is_pneu_2_extended);
		racket_pneu_2_start_time = get_full_ticks();
		
	}
	
}

