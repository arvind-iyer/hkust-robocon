/**
  ******************************************************************************
  * @file    velocity.c
  * @author  William LEE
  * @version V3.5.0
  * @date    20-January-2015
  * @brief   This file provides all velocity and pid control of motor.
  ******************************************************************************
  * @attention
  *
  * Unless specify, all "current" means at this moment, but NOT electrical 
	* current! Misunderstanding may cause series problem of using this file!
	* Default value of static variables are initialize value.
	* Approximate velocity limit is about 150.
	*
  ******************************************************************************
  */

#include "velocity.h"

static s32 current_vel = 0;					// default velocity is 0
static s32 target_vel = 0;					// default target is 0
static u16 current_accel = 100;			// default accel is 100 v/s
static s32 curr_pwm = 0;						// pwm that already set to motor
static CLOSE_LOOP_FLAG current_flag = OPEN_LOOP;	//default loop flag is open loop

/*** These will be set for once only ***/
/** 
	* @brief 		Set accleration to motor
	* @param		a, accleration (unit: velocity per second)
  * @retval 	None
	*/
void set_acceleration(u16 a)
{
	// Minimum acceleration
	if (a < MINIMUM_ACCEL) {
		a = MINIMUM_ACCEL;
	} else if (a > MAXIMUM_ACCEL) {
		a = MAXIMUM_ACCEL;
	}
	current_accel = a;
}

/**
	* @brief 		Set velocity to motor
	* @param 		vel, velocity (unit: encoder count per ms)
  * @retval 	None
	*/
void set_velocity(s32 vel)
{
	target_vel = vel;
	current_flag = CLOSE_LOOP;
}

/**
	* @brief 		Set velocity to motor
	* @param 		pwm, pwm value that are assigned
  * @retval 	None
	*/
void set_pwm(s32 pwm)
{
	curr_pwm = pwm;	
	current_flag = OPEN_LOOP;
}

/**
	* @brief 		Set all velocity as zero to force stop
	* @param 		None
  * @retval 	None
	*/
void lock_motor(void)
{
	current_vel = target_vel = 0;
}


/*** These will be call regularly ***/
/**
	* @brief 		Increase speed with respect to acceleration
							(call more frequently for higher acceleration)
	* @param 		None
  * @retval 	None
	*/
void velocity_update(void)
{
	if (current_vel > target_vel) {
		--current_vel;	// decel
	} else if (current_vel < target_vel) {
		++current_vel;	// accel
	}
}
/**
	* @brief 		Control motor with given PID in every 200 mu seconds (in systicks)
	* @param 		p, i, d:	cutomized pid valude, see PID control
  * @retval 	None
	*/
void motor_control(s32 p, s32 i, s32 d)
{
	static s32 prev_error = 0;
	static s32 last_prev_error = 0;
	const u16 PID_Prescalar = 100;		// All PID's value are multiplyed by 100
	s32 PID = p*(cal_vel_err() - prev_error) + i*cal_vel_err() + d*(cal_vel_err() + last_prev_error - prev_error*2);
	PID /= PID_Prescalar;
	
	// Disable PID control if open loop
	if (current_flag == CLOSE_LOOP) {
		curr_pwm += PID;
	} else {
		// No PID control velocity, velocity will be the differentiate of encoder value.
		target_vel = current_vel = get_encoder_vel();
	}
	
	last_prev_error = prev_error;
	prev_error = cal_vel_err();
	
	// set pwm here.
	motor_set_pwm(curr_pwm);
}


/** @brief 		Calculate velocity error between curr_vel and actual vel (private function)
	*	@param		None
	* @retval	 	the velocity value set by user
	*/
static s32 cal_vel_err(void)
{
	 return current_vel - get_encoder_vel();
}

/*** Still for external use, to be improved ***/
/** @brief	 	Get velocity value outside this file
	* @retval 	The velocity value set by user
	* @example 	Ignore CAN signal if same velocity is being set
	*/
s32 get_target_vel(void)
{
	return target_vel;
}

/** @brief 		Get acceleration value outside this file
	*	@param		None
	* @retval 	The acceleration value set by user
	* @example 	Control how frequently update the acceleration
	*/
u16 get_current_accel(void)
{
	return current_accel;
}

/** @brief		Get current loop flag outside this file
	*	@param		None
	* @retval 	The velocity value set by user
	* @example 	Ignore CAN signal if same velocity is being set
	*/
CLOSE_LOOP_FLAG get_curr_loop_flag(void)
{
	return current_flag;
}
