#ifndef __RACKET_CONTROL_H
#define __RACKET_CONTROL_H

#include "stm32f10x.h"
#include "can_motor.h"
#include "ticks.h"
#include "special_char_handler.h"
#include "ultrasonic_mb.h"
#include "stdbool.h"

#define	SWITCH_TIMEOUT					15
#define	PNEUMATIC_OPEN_DELAY	  10
#define MOTOR_OPEN_DELAY        10
#define ENCODER_THRESHOLD				10000

#define UPPER_RACKET_SENSE_DELAY		150

void racket_init(void);
void racket_update(void);

void up_racket_sensor_check(void);
void up_racket_update(void);

// functions for xbox
void open_pneumatic(void);
void close_pneumatic(void);
void serving(void);
void racket_calibrate(void);


u8 get_switch(void);
s32 get_calibrated(void);
s32 get_current_speed(void);
s32 get_turn_encoder_value(void);
s32 get_prev_encoder_value(void);
u16 get_racket_speed(void);
u16 get_racket_delay(void);
u8 get_up_switch(void);
u8 get_switch_trigger_number(void);
int get_global_delay(void);

void upper_hit(void);
void upper_hit_delay(u16 delay);
void up_racket_update(void);

void enable_ultrasonic_sensor(void);
void disable_ultrasonic_sensor(void);

void open_upper_pneumatic(void);
void close_upper_pneumatic(void);
u8 get_hitting(void);

#endif
