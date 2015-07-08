
#include "serve.h"


//racket variables
static s32 SERVE_CAL_VEL = -260;		
static s32 SERVE_HIT_VEL[SERVE_SET_COUNT] = {1300};			//can be changed by controller
static u32 SERVE_DELAY[SERVE_SET_COUNT] = {280};			// can be changed by controller
static u8  SERVE_ID = 0;

static s32 init_encoder_reading = 8000;	// will be kept updating according to the switch. Encoder value at switch location.
u32 prev_encoder_reading=0;

// timer and encoder variables
static u32 serve_start_time=0;
static u32 serve_hit_start_time = 0;
static u32 serve_calibrate_start_time = 0;
static u16 switch_counter = 0;

//States : stationary motor
bool serve_hit_queued=0;		// que for serve, and wait for delay
bool calibrated=0;		// true if calibrated already

//States : moving motor
bool hitting=0;		// true if it is hitting
bool calibrate_in_process = 0;			// true if it is calibrating


// other trigger values
bool init_encoder_is_set=0;	// first calibration is done.
bool is_released=0;	// pneumatic for serve

// AUTO SERVE VARIABLES
bool auto_serve_queued = 0;			// when you press auto-serve

// Encoder_state
bool encoder_failed=0;


void serve_timer_init(void)
{

}

// private functions

// locks motor immediately
void racket_lock()
{
	hitting=0;
	calibrate_in_process=0;
	if (!encoder_failed)
		motor_lock(RACKET);
	else
		motor_set_vel(RACKET, SERVE_CAL_VEL+100, OPEN_LOOP);
	//log("motor_lock",1);
}


void serve_pneu_set(u8 id, u8 flag)
{
	if (id >= SERVE_SET_COUNT) {return;}
	gpio_write((id == 0) ? SERVE_PNEU0_GPIO : SERVE_PNEU1_GPIO, (BitAction) flag);
	is_released = flag;
}

void serve_pneu_toggle(void)
{
	if (hitting) {return;} 
	is_released = !is_released;
	serve_pneu_set(0, !is_released);
	serve_pneu_set(1, !is_released);
}

// serve update, included in racket update
void serve_update(void)
{
	/**
	*			Start reading the pulse width
	*
	*/
	if (gpio_read_input(SERVE_SWITCH))
	{
		switch_counter++;
	}
	else
	{
		switch_counter=0;
	}
	
	
	/**
	*			Auto_serve start mechanism
	*/
	//led_control(LED_D2, (LED_STATE) !gpio_read_input(&PC6));
	if (!hitting && nec_get_msg(0)->address==0x40 && nec_get_msg(0)->command==0x01 && !calibrate_in_process && !serve_hit_queued)
	{
		start_auto_serve();
	}
	if (auto_serve_queued && calibrated)
	{
		serve_start(0);
		auto_serve_queued=0;
	}
	
	/**
	*		Serve start mechanism
	*/
	// wait for serve que, and hit the racket
	if (serve_hit_queued && get_full_ticks()>=serve_start_time+SERVE_DELAY[SERVE_ID])
	{
		buzzer_play_song(CLICK, 100, 0);
		serve_hit();
		log("serve_hit",serve_hit_start_time);
	}
	
	
	/**
	*		Serve termination mechanisms
	*/
	// while hitting, check for encoder value or time and stop.
	if ( hitting && init_encoder_is_set && !encoder_failed && (get_encoder_value(RACKET) <= init_encoder_reading+ENCODER_THRESHOLD/* ||get_full_ticks()>=serve_hit_start_time+SERVE_HIT_TIMEOUT+100-(SERVE_HIT_VEL/5)*/))
	{
		//FAIL_MUSIC;
		//motor_set_vel(RACKET, 0, OPEN_LOOP);
		hitting=0;
		calibrate_in_process=0;
		calibrated=0;
		serve_calibrate();
		log ("*enc st hit",get_encoder_value(RACKET));
	}
	// after SERVE_HIT_TIMEOUT, stop motor
	if (hitting && get_full_ticks()>=serve_hit_start_time+SERVE_HIT_TIMEOUT)
	{
		//FAIL_MUSIC;
		hitting=0;
		calibrate_in_process=0;
		calibrated=0;
		serve_calibrate();
		log("*!tim st hit",get_full_ticks() - serve_start_time);
	}

	/**
	* Mechanisms to detect encoder failure during serve
	*/
	if ((hitting || calibrate_in_process) && get_full_ticks()%20==0 && ((hitting && get_full_ticks()<=serve_hit_start_time+(SERVE_HIT_TIMEOUT*5)/4)||calibrate_in_process) )
	{
		if (!encoder_failed)
		{
			if (prev_encoder_reading==get_encoder_value(RACKET))
			{
				encoder_failed=1;
				if (hitting)
				{
					motor_set_vel(RACKET,(SERVE_HIT_VEL[SERVE_ID]*11)/10,OPEN_LOOP);
				}
				FAIL_MUSIC;
			}
		}
		else
		{
			if (prev_encoder_reading!=get_encoder_value(RACKET))
			{
				encoder_failed=0;
				CLICK_MUSIC;
			}
			
		}
		prev_encoder_reading=get_encoder_value(RACKET);
		
		
	}
	
	
	
	/**
	*		Calibration termination mechanisms
	*/
	// after calibrating, wait for switch and lock motor. Once calibrated once already, calibrate relies on encoder value to stop.
	if (calibrate_in_process && switch_counter>5)
	{
		init_encoder_is_set=1;
		calibrated=1;
		racket_lock();
		buzzer_play_song(START_UP, 120, 0);
		//ONLY UPDATE ENCODER VALUE IF RACKET HITS SWITCH
		init_encoder_reading = get_encoder_value(RACKET);
		log("*!sw st cal",get_encoder_value(RACKET));
	}
	
	// if calibration doesn't stop 1.5 seconds, FORCE STOP CALIBRATION, and register current encoder value as init encoder value.
	if (calibrate_in_process && serve_calibrate_start_time+1500<get_full_ticks())
	{
		init_encoder_is_set=0;
		SUCCESSFUL_MUSIC;
		calibrated=1;
		hitting=0;
		calibrate_in_process=0;
		motor_set_vel(RACKET, SERVE_CAL_VEL+100, OPEN_LOOP);
		//racket_lock();
		//init_encoder_reading = get_encoder_value(RACKET);
		log("*tim st cal",get_full_ticks() - serve_start_time);
	}
}


void serve_free(void)
{
	calibrated=0;
	hitting=0;
	calibrate_in_process=0;
	motor_set_vel(RACKET, 0, OPEN_LOOP);
}

void serve_start(u8 id)
{
	if (id >= SERVE_SET_COUNT) {return;} 
	
	if (!is_released && calibrated && !serve_hit_queued && !hitting && !calibrate_in_process)
	{
		serve_hit_queued = 1;
		SERVE_ID = id;
		
		serve_pneu_set(0, false);
		serve_pneu_set(1, false);
		serve_pneu_set(SERVE_ID, true);
		
		serve_start_time = get_full_ticks();
	}
}

void serve_calibrate(void)
{
	if (!calibrate_in_process && !hitting && !serve_hit_queued)
	{
		// Close all valves
		serve_pneu_set(0, false);
		serve_pneu_set(1, false);
		
		serve_calibrate_start_time=get_full_ticks();
		motor_set_vel(RACKET, SERVE_CAL_VEL, OPEN_LOOP);	//racket calibrate function takes a direct control over the motor
		calibrate_in_process=1;
		
	}
}

/*
void toggle_serve_pneu(void)
{
	//gpio_write(SERVE_PNEU_GPIO, !is_released);
	//gpio_write(SERVE_PNEU2_GPIO, !is_released);
	//gpio_write(SERVE_PNEU_GPIO_BACKUP, !is_released);
	is_released=!is_released;
	//log((is_released?"ball release! ":"hold ball"),0);
}
*/



void serve_hit(void)
{
	if (serve_hit_queued)
	{
		serve_pneu_set(0, false);
		serve_pneu_set(1, false);
		
		hitting = 1;
		calibrated=0;
		serve_hit_queued=0;
		if (!encoder_failed)
			motor_set_vel(RACKET, SERVE_HIT_VEL[SERVE_ID]/10,CLOSE_LOOP);	//racket calibrate function takes a direct control over the motor
		else
			motor_set_vel(RACKET,(SERVE_HIT_VEL[SERVE_ID]*21)/20,OPEN_LOOP);
		serve_hit_start_time = get_full_ticks();
	}
	
}
void fake_serve_start(void)
{
	if (!is_released && calibrated && !serve_hit_queued && !hitting && !calibrate_in_process)
	{
		hitting=1;
		calibrated=0;
		serve_hit_queued=0;
		if (!encoder_failed)
			motor_set_vel(RACKET, SERVE_HIT_VEL[SERVE_ID]/10,CLOSE_LOOP);	//racket calibrate function takes a direct control over the motor
		else
			motor_set_vel(RACKET,(SERVE_HIT_VEL[SERVE_ID]*21)/20,OPEN_LOOP);
		serve_hit_start_time=get_full_ticks();
		
	}
	
}

void start_auto_serve(void)	// auto calibration before serve.
{
	if (!hitting && !calibrate_in_process && !serve_hit_queued)
	{
		auto_serve_queued=1;
		if (!calibrated){serve_calibrate();}
		else {serve_start(0);
		auto_serve_queued=0;}
		
	}
	
	
}
/*************getter and setter functions ***********************/

void serve_change_delay(u8 id, s16 val)
{
	if (id >= SERVE_SET_COUNT) {return;}
	SERVE_DELAY[id]+=val;
}

void serve_change_vel(u8 id, s16 val)
{
	if (id >= SERVE_SET_COUNT) {return;}
	SERVE_HIT_VEL[id]+=val;
}

void serve_set_delay(u8 id, s16 delay)
{
	if (id >= SERVE_SET_COUNT) {return;}
	SERVE_DELAY[id] = delay;
}

void serve_set_vel(u8 id, s16 vel)
{
	if (id >= SERVE_SET_COUNT) {return;}
	SERVE_HIT_VEL[id] = vel;
}

s32 serve_get_vel(u8 id)
{
	if (id >= SERVE_SET_COUNT) {return 0;}
	return SERVE_HIT_VEL[id];
}

u32 serve_get_delay(u8 id)
{
	if (id >= SERVE_SET_COUNT) {return 0;}
	return SERVE_DELAY[id];
}

bool serve_prioritized(void)
{
	#if (ROBOT == 'D')
		return (serve_hit_queued || hitting || calibrate_in_process);
	#elif
		return false;
	#endif
}
