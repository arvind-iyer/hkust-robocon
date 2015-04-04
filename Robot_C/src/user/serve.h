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

#define RACKET MOTOR5
#define SERVE_PNEU_GPIO &PE7


#define ENCODER_THRESHOLD 	12500


void serve_update();

// interface
void serve_free(void);
void serve_lock(void);
void serve_calibrate(void);
void serve_hit(void);
void serve_start(void);

// primary control
void toggle_serve_pneu(void);


// getter and setter for properties
void serve_change_vel(s16);
s32 serve_get_vel(void);
void serve_change_delay(s16);
u32 serve_get_delay(void);
//s32 get_init_enc(void);
//s32 serve_get_last_stop_encoder_value(void);
#endif
