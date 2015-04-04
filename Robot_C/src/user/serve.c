#include "serve.h"


//racket variables
static s32 RACKET_CAL_VEL = 9 ;		
static s32 RACKET_HIT_VEL = -1350;			//can be changed by controller
static u32 RACKET_SERVE_DELAY = (ROBOT == 'C' ? 1200 : 510);			// can be changed by controller


static s32 init_encoder_reading = -5000;

// timer and encoder variables
static u32 racket_serve_start_time=0;
static u32 racket_serve_end_time = 0;
static s32 racket_last_stop_encoder_value=0;




void serve_update(void)
{
	
	
}


void serve_free(void)
{
	//is_locked = 0;
	//racket_hit_disable();
	motor_set_vel(RACKET, 0, OPEN_LOOP);
}

void serve_start(void)
{
	
}

void serve_calibrate(void)
{
	if (0/*!is_locked && *//*!button_pressed(ROTATE_SWITCH)*/)
	{
		motor_set_vel(RACKET, RACKET_CAL_VEL, CLOSE_LOOP);	//racket calibrate function takes a direct control over the motor
		//racket_hit_disable();
	}
}

void toggle_serve_pneu(void)
{
	//servo_control(SERVO4, (is_servo_release ? 900 : 1500));
	//is_servo_release = !is_servo_release;
}

void serve_hit(void);

// interface


/*************getter and setter functions ***********************/
s32 racket_get_last_stop_encoder_value(void)
{
	return racket_last_stop_encoder_value;
}

void serve_change_delay(s16 val)
{
	RACKET_SERVE_DELAY+=val;
}

void serve_change_vel(s16 val)
{
	RACKET_HIT_VEL-=val;
}

s32 serve_get_vel(void)
{
	return RACKET_HIT_VEL;
}

u32 serve_get_delay(void)
{
	return RACKET_SERVE_DELAY;
}