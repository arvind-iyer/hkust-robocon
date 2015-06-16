/**
  ******************************************************************************
  * @file    velocity.c
  * @author  William LEE
  * @version V3.5.0
  * @date    09-June-2015
  * @brief   This file provides all velocity and pid control of motor.
  ******************************************************************************
  * @attention
  *
  * Unless specify, all "current" means at this moment, but NOT electrical 
	* current! Misunderstanding may cause series problem of using this file!
	* Default value of static variables are initialize value.
	* Approximate velocity limit is about 165.
	*
  ******************************************************************************
  */

#include "velocity.h"

// Constant value
const u16 PID_Prescalar = 10000;		// All PID's value are multiplyed by 10000
const u16 ACCEL_PRESCALAR = 1000;		// convert v/s to v/ms, while v means velocity

// Static variable that record current and target state (initiated as default value).
static s32 current_vel = 0;					// default velocity is 0
static s32 target_vel = 0;					// default target is 0
static u16 current_accel = 100;			// default accel is 100 v/s
static s32 curr_pwm = 0;						// pwm that already set to motor
static CLOSE_LOOP_FLAG current_flag = OPEN_LOOP;
static bool encoder_working = true;

// Static variable for position mode
static PID_MODE curr_pid_mode = VEL_MODE;
static s32 targete_pos = 0;
static u16 max_speed = 0;


/*** These will be set for once only ***/
/** 
	* @brief 		Set accleration to motor
	* @param		a, accleration (unit: velocity per second)
  * @retval 	None
	*/
void set_acceleration(u16 a)
{
	const u16 MINIMUM_ACCEL = 2;
	const u16 MAXIMUM_ACCEL = 1000;

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
	curr_pid_mode = VEL_MODE;
}

/**
	* @brief 		Set velocity to motor
	* @param 		pos: The encoder value that you want to stop
							spd: The max speed that motor can reach to lock the position.
  * @retval 	None
	*/
void set_pos(s32 pos, u16 spd)
{
	targete_pos = pos;
	if (max_speed < 150) {
		max_speed = spd;
	} else {
		max_speed = 150;
	}
	current_flag = CLOSE_LOOP;
	curr_pid_mode = POS_MODE;
}


/**
	* @brief		Control the position (absolute position).
	* @param		None
	* @retval		None
	*/
void pos_control(void)
{
	// Max speed can only be 150, so the dead zone always < its square (22500).
	// You can use decrease the speed or increase the acceleration to minimize the dead zone.
	if (Abs(get_encoder() - targete_pos) > Sqr(current_vel) * ACCEL_PRESCALAR / current_accel) {
		set_velocity(max_speed);
	} else {
		set_velocity(0);
	}
}

/**
	* @brief 		Set velocity to motor
	* @param 		pwm, pwm value that are assigned
  * @retval 	None
	*/
void set_pwm(s32 pwm)
{
	curr_pwm = pwm * PID_Prescalar;	
	current_flag = OPEN_LOOP;
	curr_pid_mode = VEL_MODE;
}

/**
	* @brief 		Set all velocity as zero to force stop
	* @param 		None
  * @retval 	None
	*/
void lock_motor(void)
{
	current_vel = target_vel = 0;
	current_flag = CLOSE_LOOP;
	curr_pid_mode = VEL_MODE;
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
	static s32 accumulated_remainder = 0;
	if (Abs(target_vel - current_vel) * 1000 > current_accel) {
		s16 dir = (target_vel - current_vel) / Abs(target_vel - current_vel);
		current_vel +=  (current_accel * dir) / ACCEL_PRESCALAR;
		accumulated_remainder += (current_accel * dir) % ACCEL_PRESCALAR;
	} else {
		// No carry from remainder if it already reach target.
		current_vel = target_vel;
		accumulated_remainder = 0;
		return;
	}
	
	// Update remainder and carry to velocity if remainder is large enough (just simulating floating point).
	if (Abs(accumulated_remainder) > ACCEL_PRESCALAR) {
		s32 carry_to_vel = accumulated_remainder / ACCEL_PRESCALAR;
		if (carry_to_vel != 0) {
			accumulated_remainder -= carry_to_vel * ACCEL_PRESCALAR;
			current_vel += carry_to_vel;
		}
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
//	static s32 lock_time_out = 0;
  static u16 encoder_fail_time_out = 0;
	
//	if (curr_pid_mode == POS_MODE) {
//		pos_control();
//	}
//	
	s32 PID = p*(cal_vel_err() - prev_error) + i*cal_vel_err() + d*(cal_vel_err() + last_prev_error - prev_error*2);
	
	if (current_flag == CLOSE_LOOP && encoder_working) {
    // Disable close loop if velocity is set but no change in encoder / unexpected full power.
    if (Abs(current_vel) > 0 && get_encoder_vel() == 0) {
      ++encoder_fail_time_out;
      if (encoder_fail_time_out > 100) {
        encoder_working = false;
      }
    } else {
      encoder_fail_time_out = 0;
    }
    // Silent the motor, if motor is static but pwm is still not 0. (e.g.: 0~10 response nothing to encoder)
		// This part have nearly no effect as it will affect the locking performance, just left it here.
//    if (curr_pwm != 0 && current_vel == 0 && PID == 0) {
//      ++lock_time_out;
//      if (lock_time_out > 1000) {
//        // Minimize the pwm when static without affect it performance suddenly..
//        if (curr_pwm > 0) {
//          --curr_pwm;
//        } else if (curr_pwm < 0) {
//          ++curr_pwm;
//        }
//				lock_time_out = 0;
//      }
//    } else {
      // Use pwm control and reset timeout.
		curr_pwm += PID;
//      lock_time_out = 0;
//    }
	} else {
    // Disable PID control if encoder is not work, no pwm to motor and warning light.
    if (current_flag == CLOSE_LOOP && !encoder_working) {
      curr_pwm = 0;
    }

		// No PID control velocity, velocity will be the differentiate of encoder value.
    if (encoder_working) {
      target_vel = current_vel = get_encoder_vel();
    }
    
    // Enable PID control again if the encoder is working
    if (!encoder_working && get_encoder_vel() != 0) {
      encoder_fail_time_out = 0;
      encoder_working = true;
    }
	}
	
	last_prev_error = prev_error;
	prev_error = cal_vel_err();
	
	// Output to motor
	motor_set_pwm(curr_pwm / PID_Prescalar);
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

/** @brief  Check encoder working or not
  * @param  None
  * @retval True if encoder is working and false otherwise.
  */
bool is_encoder_working(void)
{
  return encoder_working;
}

PID_MODE get_curr_pid_mode(void)
{
	return curr_pid_mode;
}
