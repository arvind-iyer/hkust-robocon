#ifndef __RACKET_CONTROL_H
#define __RACKET_CONTROL_H

#include "stm32f10x.h"
#include "can_motor.h"
#include "ticks.h"
#include "special_char_handler.h"
#include "stdbool.h"

#define	SWITCH_TIMEOUT					15
#define	PNEUMATIC_OPEN_DELAY	  10
#define MOTOR_OPEN_DELAY        10
#define ENCODER_THRESHOLD				10000

void racket_init(void);
void racket_update(void);
void up_racket_update(void);

// functions for xbox
void open_pneumatic(void);
void close_pneumatic(void);
void serving(void);
void racket_calibrate(void);


u8 get_switch(void);
s32 get_calibrated(void);
s32 get_current(void);
s32 get_prev(void);
u16 get_racket_speed(void);
u16 get_racket_delay(void);
u8 get_up_switch(void);

#endif
