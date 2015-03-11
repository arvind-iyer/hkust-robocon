#include "racket.h"



//racket variables
static s32 RACKET_CAL_VEL = 9 ;		
static s32 RACKET_HIT_VEL = -300;//-1320;				//can be changed by controller
static u32 RACKET_SERVE_DELAY = (ROBOT == 'C' ? 1200 : 495);			// can be changed by controller
static s32 init_encoder_reading = -5000;



// flags
static u8 hit_in_progress = 0;				// true if racket hit is in progress. Should only be used for temporarily disabling mechanical switch
static u8 is_locked = 0;							// true iff racket is locked by mechanical switch by calibration
static u8 is_servo_release = 0;				// true if servo is at release state, vice versa
static bool is_pneu_extended = 1; 			// true if pneumatic piston is extended
//triggers
static u8 serve_enabled=0;					// flag is raised and waits for timer to trigger racket_hit()
static u8 racket_laser_not_alligned = 1;		// true if the robot detects error in laser allignment
//static u8 racket_laser_trigger_enabled = (ROBOT=='C'? 0: 1);		// flag is raised and waits for timer to trigger racket_hit()


// timer and encoder variables
static u32 racket_serve_start_time=0;
static u32 racket_serve_end_time = 0;
static u32 racket_last_laser_trigger_time=0;
static u32 racket_laser_trigger_interval=1000;
static u32 racket_pneu_start_time = 0;

// test variables
static s32 racket_last_stop_encoder_value=0;


/*************getter and setter functions ***********************/
s32 racket_get_last_stop_encoder_value(void)
{
	return racket_last_stop_encoder_value;
}
s32 racket_get_laser_hit_delay(void)
{
	return racket_laser_trigger_interval;
}
void racket_change_laser_hit_delay(s16 val)
{
	racket_laser_trigger_interval += val;
}

void racket_change_serve_delay(s16 val)
{
	RACKET_SERVE_DELAY+=val;
}

void racket_change_hit_vel(s16 val)
{
	RACKET_HIT_VEL-=val;
}

s32 racket_get_vel()
{
	return RACKET_HIT_VEL;
}

u32 racket_get_serve_delay()
{
	return RACKET_SERVE_DELAY;
}

u8 get_lock_status(void)
{
	return is_locked;
}

s32 get_init_enc(void)
{
	return init_encoder_reading;
}

void racket_hit_initiated(void)
{
	hit_in_progress = 1;
}

void racket_hit_disable(void)
{
	hit_in_progress = 0;
}




/********************** here all getters and setters ends **************************/



void racket_pneumatic_set(bool data)		// 
{
	gpio_write(PNEU_GPIO, !data);
	gpio_write(PNEU_GPIO_DOWN, !data);
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
	if(is_pneu_extended && (get_full_ticks() > racket_pneu_start_time + RACKET_SERVE_DELAY))
	{
		is_pneu_extended = 0;
		racket_pneumatic_set(is_pneu_extended);
	}
	if (/*ROBOT=='D' && */!gpio_read_input(LASER_GPIO) && racket_laser_not_alligned)
	{
		racket_laser_not_alligned=0;
	//	racket_hit();
		serve_enabled = 0;
		racket_last_laser_trigger_time=get_full_ticks();

	}
	
	if (/*ROBOT=='D' && */!racket_laser_not_alligned && gpio_read_input(LASER_GPIO) && get_full_ticks() > racket_last_laser_trigger_time + racket_laser_trigger_interval)		//trigger racket with laser
	{
		////serve_enabled = 0;
		
		serve_enabled = 0;
		racket_last_laser_trigger_time=get_full_ticks();
		racket_hit();
		racket_laser_not_alligned=1;
		
		
		////racket_laser_trigger_enabled=0;
	}
	
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
	
	if(racket_laser_not_alligned && !hit_in_progress && (button_pressed(RACKET_SWITCH) || button_pressed(ROTATE_SWITCH)) )		// if any of the mechanical switch is pressed, lock the motor.
	{
			is_locked = 1;
			motor_lock(RACKET);
	}
	
	
}


void racket_stop(void)
{
	is_locked = 0;
	racket_hit_disable();
	motor_set_vel(RACKET, 0, OPEN_LOOP);
}

void racket_calibrate(void)
{
	if (/*!is_locked && */!button_pressed(ROTATE_SWITCH))
	{
		motor_set_vel(RACKET, RACKET_CAL_VEL, CLOSE_LOOP);	//racket calibrate function takes a direct control over the motor
		racket_hit_disable();
	}
}

void toggle_servo(void)
{
	servo_control(SERVO4, (is_servo_release ? 900 : 1500));
	is_servo_release = !is_servo_release;
}

void racket_hit(void)
{
	if(ROBOT == 'D')
	{	
		if(button_pressed(ROTATE_SWITCH))//pe3
		{
			init_encoder_reading =  get_encoder_value(RACKET);
			racket_hit_initiated();
			is_locked = 0;
			motor_set_vel(RACKET, RACKET_HIT_VEL, OPEN_LOOP);		//racket hit function takes a direct control over the motor
		}
		else
		{
			racket_calibrate();
			
		}	
	}
	else if (!is_pneu_extended)
	{
		is_pneu_extended = 1;
		racket_pneumatic_set(is_pneu_extended);
		racket_pneu_start_time = get_full_ticks();
		
	}
	
}

void racket_start_serve(void)
{
	if (!is_servo_release)
	{
		toggle_servo();
		racket_serve_start_time = get_full_ticks();
		serve_enabled=1;
	}
}
