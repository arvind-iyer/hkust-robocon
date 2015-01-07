#ifndef __DRIVE_H
#define __DRIVE_H

#include <stdio.h>
#include "stm32f10x.h"
/*** Essential ***/
#include "debug.h"
#include "lcd_red.h"
#include "uart.h"
#include "approx_math.h"
#include "motor_pwm.h"
#include "lm629.h"
#include "gyro.h"
#include "light_sensor.h"

#include "button.h"
/*** Variable ***/

#define 	FRONT_LEFT_MOTOR 		0
#define 	FRONT_RIGHT_MOTOR		3
#define 	BACK_RIGHT_MOTOR 		1
#define		BACK_LEFT_MOTOR		2

extern u16 ticks_img;
extern u16 seconds_img;


/*** Function ***/
void set_motor_rel(s32 x_vel, s32 y_vel, s32 w_vel);
void manual_control(void);
u8 auto_control(s32 target_x, s32 target_y, s32 target_angle);
void auto_test(void);

#endif /* __DRIVE_H */
