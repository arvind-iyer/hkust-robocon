#ifndef __RACKET_CONTROL_H
#define __RACKET_CONTROL_H

#include "stm32f10x.h"
#include "can_motor.h"
#include "ticks.h"
#include "special_char_handler.h"
#include "stdbool.h"

// Sensor GPIO Pin

// Button Switch Pin
#define Switch_B1_Pin  GPIO_Pin_0
#define Switch_B2_Pin  GPIO_Pin_1
#define Switch_Low_Pin  Switch_B2_Pin
#define Switch_High_Pin Switch_B1_Pin

#define LOW_RACKET_MOTOR MOTOR6
#define LOW_RACKET_HIT_SPEED -1800

// All racket functions
void racket_init(void);

// Lower racket functions
void low_racket_move(void);
u32 low_racket_move_time(void);
void low_racket_stop(void);
void low_racket_standby(void);
void low_racket_update(void);

// Higher racket functions
//void high_racket_move(void);
//void high_racket_update(void);

void racket_received_command(void);
bool did_receive_command(void);
void racket_update(void);

s32 get_low_speed(void);
s32 get_high_speed(void);

bool get_low_switch(void);
bool get_high_switch(void);

u8 get_high_mode(void);
u8 get_low_mode(void);

void add_racket_speed(void);
void decrease_racket_speed(void);
u16 get_racket_speed(void);

s32 racket_current_time(void);
u32 high_racket_move_time(void);

void racket_out(void);
void racket_in(void);

#endif
