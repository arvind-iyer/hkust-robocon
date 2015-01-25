#ifndef VELOCITY_H
#define VELOCITY_H

#include "stm32f10x.h"
#include "approx_math.h"
#include "motor.h"
#include "encoder.h"

#define ENCODER_TIM TIM2
#define MINIMUM_ACCEL 2
#define MAXIMUM_ACCEL 1000

typedef enum {
	OPEN_LOOP = 0,
	CLOSE_LOOP = 1
} CLOSE_LOOP_FLAG;


/*** these are regularly call ***/
void velocity_update(void);
void motor_control(s32 p, s32 i, s32 d);

/*** these are triggered by CAN once receive ***/
void lock_motor(void);
void set_acceleration(u16 a);
void set_velocity(s32 vel);
void set_pwm(s32 pwm);

/*** Internal used ***/
static s32 cal_vel_err(void);

/*** still use, wait for improvement ***/
s32 get_target_vel(void);
s32 get_encoder(void);
u16 get_current_accel(void);
CLOSE_LOOP_FLAG get_curr_loop_flag(void);

#endif	//VELOCITY_H
