#ifndef __RACKET_CONTROL_H
#define __RACKET_CONTROL_H

#include "stm32f10x.h"
#include "can_motor.h"
#include "ticks.h"
#include "special_char_handler.h"
#include "stdbool.h"

// Sensor GPIO Pin
#define IR_Sensor_1_Pin GPIO_Pin_5 // A5
#define IR_Sensor_2_Pin GPIO_Pin_6 // A6
#define IR_Sensor_3_Pin GPIO_Pin_7 // A7

// Button Switch Pin
#define Switch_B1_Pin  GPIO_Pin_0
#define Switch_B2_Pin  GPIO_Pin_1
#define Switch_Low_Pin  Switch_B2_Pin
#define Switch_High_Pin Switch_B1_Pin

#define LOW_RACKET_MOTOR MOTOR6
#define LOW_RACKET_HIT_SPEED -1800

// Sensor functions
void sensor_init(void);
void sensor_update(void);

// All racket functions
void racket_init(void);

// Lower racket functions
void low_racket_move(void);
u32 low_racket_move_time(void);
void low_racket_stop(void);
void low_racket_standby(void);
void low_racket_update(void);

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

bool where_should_racket_stop(void);
void racket_keep_low(void);
void racket_keep_high(void);

#endif
