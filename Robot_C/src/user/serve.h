#ifndef __SERVE_H
#define __SERVE_H
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "special_char_handler.h"
#include "button.h"
#include "can_motor.h"
#include "approx_math.h"
#include "servo.h"
#include "gpio.h"
#include "robocon.h"

#define SERVE_SWITCH &PE11
#define RACKET MOTOR5
#define SERVE_PNEU_GPIO &PE15//&PD9
#define SERVE_PNEU_GPIO_BACKUP &PD9

#define ENCODER_THRESHOLD 	-10000

static u32 SERVE_HIT_TIMEOUT = 130;	// maximum serve duration.

void serve_update(void);

// interface
void serve_free(void);
void serve_lock(void);
void serve_calibrate(void);		// THIS IS ALSO THE FUNCTION U CALL
void serve_hit(void);		// DO NOT CALL THIS FUNCTION
void serve_start(void);		// THIS IS FUNCTION U CALL
void fake_serve_start(void);

// primary control
void toggle_serve_pneu(void);


// getter and setter for properties
void serve_change_vel(s16);
s32 serve_get_vel(void);
void serve_change_delay(s16);
u32 serve_get_delay(void);
bool serve_prioritized(void);
#endif
