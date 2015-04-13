#include "serve.h"


//racket variables
static s32 SERVE_CAL_VEL = -200 ;		
static s32 SERVE_HIT_VEL = 1300;			//can be changed by controller
static u32 SERVE_DELAY = 300;			// can be changed by controller

static s32 init_encoder_reading = 8000;	// will be kept updating according to the switch. Encoder value at switch location.
static s32 prev_encoder_reading = 8000;	// encoder reading when racket is locked.

// timer and encoder variables
static u32 serve_start_time=0;
static u32 serve_hit_start_time = 0;
static u32 serve_calibrate_start_time = 0;


//States : stationary motor
bool serve_hit_queued=0;		// que for serve, and wait for delay
bool calibrated=0;		// true if calibrated already

//States : moving motor
bool hitting=0;		// true if it is hitting
bool calibrate_in_process = 0;			// true if it is calibrating


// other trigger values
bool init_encoder_is_set=0;	// first calibration is done.
bool is_released=0;	// pneumatic for serve



// private functions

// locks motor immediately
void racket_lock()
{
	hitting=0;
	calibrate_in_process=0;
	prev_encoder_reading = get_encoder_value(RACKET);
	motor_lock(RACKET);
	//log("motor_lock",1);
}



// serve update, included in racket update
void serve_update(void)
{
	// wait for serve que, and hit the racket
	if (serve_hit_queued && get_full_ticks()>=serve_start_time+SERVE_DELAY)
	{
		serve_hit();
	}
	
	// while hitting, check for encoder value or time and stop.
	if ( hitting && (get_encoder_value(RACKET) <= init_encoder_reading+ENCODER_THRESHOLD/* ||get_full_ticks()>=serve_hit_start_time+SERVE_HIT_TIMEOUT+100-(SERVE_HIT_VEL/5)*/))
	{
		racket_lock();
		log ("enc st hit",get_encoder_value(RACKET));
		log ("serve delay", serve_hit_start_time - serve_start_time);
		log ("serve time", get_full_ticks() - serve_hit_start_time);
	}
	if (hitting && get_encoder_value(RACKET)==prev_encoder_reading && get_full_ticks()>=serve_hit_start_time+SERVE_HIT_TIMEOUT)
	{
		hitting=0;
		motor_set_vel(RACKET, 0, OPEN_LOOP);
		serve_calibrate();
		log("*tim st hit",get_full_ticks() - serve_start_time);
	}

	
	// after calibrating, wait for switch and lock motor. Once calibrated once already, calibrate relies on encoder value to stop.
	if (calibrate_in_process && (gpio_read_input(SERVE_SWITCH) || (init_encoder_is_set && get_encoder_value(RACKET)>init_encoder_reading-1000 )))
	{
		init_encoder_is_set=1;
		calibrated=1;
		racket_lock();
		
		// temporary code. ONLY UPDATE ENCODER VALUE IF RACKET HITS SWITCH
		if (gpio_read_input(SERVE_SWITCH))
		{
			init_encoder_reading = get_encoder_value(RACKET);
			log("*sw st cal",get_encoder_value(RACKET));
		}
		else
			log("enc st cal",get_encoder_value(RACKET));
	}
	
	
	// DUMMY CODE : PRINT OUT LOG ERROR MESSAGE IF MOTOR IS MOVING WITH FLAGS DOWN
	if (!hitting && !calibrated && (gpio_read_input(SERVE_SWITCH) || (init_encoder_is_set && get_encoder_value(RACKET)>init_encoder_reading-2000 )))
	{
		//log("CAL ERROR",gpio_read_input(SERVE_SWITCH));
	}
	
	// if calibration doesn't stop 1.5 seconds, FORCE STOP CALIBRATION, and register current encoder value as init encoder value.
	if (calibrate_in_process && serve_calibrate_start_time+1500<get_full_ticks())
	{
		init_encoder_is_set=1;
		calibrated=1;
		//racket_lock();
		prev_encoder_reading = get_encoder_value(RACKET);
		init_encoder_reading = get_encoder_value(RACKET);
		log("*tim st cal",get_full_ticks() - serve_start_time);
	}
}


void serve_free(void)
{
	calibrated=0;
	motor_set_vel(RACKET, 0, OPEN_LOOP);
}

void serve_start(void)
{
	if (!is_released && calibrated && !serve_hit_queued && !hitting && !calibrate_in_process)
	{
		serve_hit_queued = 1;
		toggle_serve_pneu();
		
		serve_start_time = get_full_ticks();
	}
}

void serve_calibrate(void)
{
	if (!calibrate_in_process && !hitting && !serve_hit_queued)
	{
		serve_calibrate_start_time=get_full_ticks();
		motor_set_vel(RACKET, SERVE_CAL_VEL, OPEN_LOOP);	//racket calibrate function takes a direct control over the motor
		calibrate_in_process=1;
		
	}
}

void toggle_serve_pneu(void)
{
	gpio_write(SERVE_PNEU_GPIO, is_released);
	is_released=!is_released;
	//log((is_released?"ball release! ":"hold ball"),0);
}

void serve_hit(void)
{
	if (serve_hit_queued)
	{
		toggle_serve_pneu();
		
		hitting = 1;
		calibrated=0;
		serve_hit_queued=0;
		motor_set_vel(RACKET, SERVE_HIT_VEL, OPEN_LOOP);	//racket calibrate function takes a direct control over the motor
		serve_hit_start_time = get_full_ticks();
	}
	
}


/*************getter and setter functions ***********************/

void serve_change_delay(s16 val)
{
	SERVE_DELAY+=val;
}

void serve_change_vel(s16 val)
{
	SERVE_HIT_VEL+=val;
}

s32 serve_get_vel(void)
{
	return SERVE_HIT_VEL;
}

u32 serve_get_delay(void)
{
	return SERVE_DELAY;
}

bool serve_prioritized(void)
{
	return ROBOT=='D' && (serve_hit_queued || hitting || calibrate_in_process);
}