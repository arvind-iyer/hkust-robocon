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
void racket_open_serve_pneumatic(void);
void racket_close_serve_pneumatic(void);
void racket_trigger_serving(void);
void racket_calibrate(void);


u8 racket_get_switch(void);
s32 racket_get_calibration_status(void);
s32 racket_get_current_speed(void);
u16 racket_get_racket_speed(void);
u16 racket_get_racket_delay(void);
bool racket_is_serving(void);

int get_global_delay(void);

void racket_upper_hit(void);
void racket_upper_hit_delay(u16 delay);
void racket_up_racket_update(void);
void racket_increase_racket_delay(void);
void racket_decrease_racket_delay(void);

void enable_ultrasonic_sensor(void);
void disable_ultrasonic_sensor(void);

void racket_open_upper_pneumatic(void);
void racket_close_upper_pneumatic(void);
u8 racket_get_hitting(void);

#endif
