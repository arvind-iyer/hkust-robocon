#include "racket.h"





// flags
//static u8 hit_in_progress = 0;				// true if racket hit is in progress. Should only be used for temporarily disabling mechanical switch
//static u8 is_locked = 0;							// true iff racket is locked by mechanical switch by calibration
//static u8 is_servo_release = 0;				// true if servo is at release state, vice versa
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

/*void is_laser_serve_enabled(u8 bit)
{
	if (bit > 1)
		return;
	racket_laser_trigger_enabled = bit;
}
*/
//Called ever 10ms to check and update racket data and redirect to starting and stopping racket at required points
void racket_update(void)
{
	/*if (ROBOT=='C' && gpio_read_input(LASER_GPIO))
	{
		racket_hit();
	}*/
	if(is_pneu_extended && (get_full_ticks() > racket_pneu_start_time + 1000/*RACKET_SERVE_DELAY*/))
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
	/*
	if (serve_enabled && get_full_ticks()>RACKET_SERVE_DELAY+racket_serve_start_time)		// execute hit_racket() after serve delay
	{
		serve_enabled=0;
		racket_hit();
		toggle_servo();
	}
	
	
	if(hit_in_progress && get_encoder_value(RACKET) > (init_encoder_reading+ENCODER_THRESHOLD))		// during racket_hit(), lock the motor if the racket pass the encoder point.
	{
		racket_last_stop_encoder_value=get_encoder_value(RACKET);
		motor_lock(RACKET);
		is_locked = 1;
		init_encoder_reading = get_encoder_value(RACKET);
		racket_hit_disable();
		
		racket_serve_end_time = get_full_ticks();
	}
	if(is_locked && (racket_serve_end_time + 400 < get_full_ticks()))
	{
		racket_serve_end_time = get_full_ticks();
		racket_calibrate();
	}
	
	if(!hit_in_progress )		// if any of the mechanical switch is pressed, lock the motor.
	{
			is_locked = 1;
			motor_lock(RACKET);
	}
	*/
	
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


