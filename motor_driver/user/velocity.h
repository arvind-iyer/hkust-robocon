#ifndef VELOCITY_H
#define VELOCITY_H

#include "stm32f10x.h"
#include "approx_math.h"
#include "motor.h"
#include "encoder.h"

#define ENCODER_TIM TIM2

typedef enum {
	POS_MODE = 0,
	VEL_MODE = 1
} PID_MODE;

typedef enum {
	OPEN_LOOP = 0,
	CLOSE_LOOP = 1
} CLOSE_LOOP_FLAG;

extern const u16 PID_Prescalar;

/*** these are regularly call ***/
void velocity_update(void);
void motor_control(s32 p, s32 i, s32 d);
void pos_control(void);

/*** these are triggered by CAN once receive ***/
void lock_motor(void);
void set_acceleration(u16 a);
void set_velocity(s32 vel);
void set_pwm(s32 pwm);
void set_pos(s32 pos, u16 spd);

/*** Internal used ***/
static s32 cal_vel_err(void);

/*** still use, wait for improvement ***/
s32 get_target_vel(void);
s32 get_encoder(void);
u16 get_current_accel(void);
CLOSE_LOOP_FLAG get_curr_loop_flag(void);
PID_MODE get_curr_pid_mode(void);
bool is_encoder_working(void);

#endif	//VELOCITY_H
