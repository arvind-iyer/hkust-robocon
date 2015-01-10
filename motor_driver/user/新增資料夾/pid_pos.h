#ifndef PID_POS_H
#define PID_POS_H

#include <stdio.h>
#include "stm32f10x.h"
#include "sysinit.h"
#include "misc.h"
#include "stm32f10x_rcc.h"

#include "main.h"
#include "motion.h"
#include "pid_vel.h"

float abs(float);

void pos_set_pid(float _p, float _i, float _d);
void pos_set_max(float _acc, float _count);
float pos_get_curr(void);
void pos_set_home(void);

void pos_err(void);

void pos_cal_function_mode(void);
void pos_move_function_mode(void);

void pos_move(void);



#endif
