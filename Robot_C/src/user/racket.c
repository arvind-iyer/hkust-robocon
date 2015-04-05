#include "racket.h"





// flags
static bool is_pneu_extended = 1; 			// true if pneumatic piston is extended
//triggers
//static u8 serve_enabled=0;					// flag is raised and waits for timer to trigger racket_hit()
//static u8 racket_laser_not_alligned = 1;		// true if the robot detects error in laser allignment
//static u8 racket_laser_trigger_enabled = (ROBOT=='C'? 0: 1);		// flag is raised and waits for timer to trigger racket_hit()


// timer and encoder variables

//static u32 racket_last_laser_trigger_time=0;
//static u32 racket_laser_trigger_interval=1000;
static u32 racket_pneu_start_time = 0;




/********************** here all getters and setters ends **************************/



void racket_pneumatic_set(bool data)		// 
{
	gpio_write(PNEU_GPIO, !data);
	gpio_write(PNEU_GPIO_DOWN, !data);
	log("pneu",is_pneu_extended);
}


//Called ever 10ms to check and update racket data and redirect to starting and stopping racket at required points
void racket_update(void)
{
	/*if (ROBOT=='C' && gpio_read_input(LASER_GPIO))
	{
		racket_hit();
	}*/
	if(is_pneu_extended && (get_full_ticks() > racket_pneu_start_time + 2000/*RACKET_SERVE_DELAY*/))
	{
		is_pneu_extended = 0;
		racket_pneumatic_set(is_pneu_extended);
		log("pneu",is_pneu_extended);
	}
	
	if (ROBOT=='D')
		serve_update();
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

void racket_hit(void)
{
	if (!is_pneu_extended)
	{
		is_pneu_extended = 1;
		racket_pneumatic_set(is_pneu_extended);
		racket_pneu_start_time = get_full_ticks();
		
	}
	
}


