#ifndef PID_VEL_H
#define PID_VEL_H

#include <stdio.h>
#include "stm32f10x.h"
//#include "sysinit.h"
//#include "misc.h"
//#include "stm32f10x_rcc.h"

//#include "main.h"
#include "motion.h"
#include "pid_pos.h"

#define MEMORY_SIZE 20
#define MAX_PWM 1799
#define MAX_CURRENT 1100
#define INTEGRAL_THRESHOLD 22

//commend only for debug
void vel_set_pid1(float _p, float _i, float _d);
void vel_set_pid2(float _p, float _i, float _d);
void vel_init_pwm(float _pwm, s8 _dir);
void vel_set_max(float m);
void vel_set_speed(float r);
float vel_get_speed(void);

//sub components
void clear_record(void);
void init_memory(void);
void vel_err(void);
void set_default_pid(void);
void set_pwm_to_motor(void);
void read_encoder(void);
void get_current(void);

//sub functions
void vel_move(u8);
void vel_stop(u8);
void vel_zero(void);

//main function
void vel_n_pos(void);

//functions called by user
void motor_set_pwm(float user_pwm);
void motor_set_pwm_current(float user_pwm);
void motor_set_speed(float speed);
void motor_lock(void);
void motor_zero(void);
void cali_user(void);
void set_acceleration(u16 value);
void increase_encoder(void);

#endif
