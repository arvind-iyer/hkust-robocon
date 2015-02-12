#ifndef __RACKET_CONTROL_H
#define __RACKET_CONTROL_H

#include "stm32f10x.h"
#include "can_motor.h"
#include "ticks.h"
#include "special_char_handler.h"
#include "stdbool.h"

#define S1_Pin	GPIO_Pin_5	// GPIOE
#define S2_Pin	GPIO_Pin_6	// GPIOE
#define B1_Pin	GPIO_Pin_13	// GPIOE
#define B2_Pin	GPIO_Pin_14 // GPIOE
#define B3_Pin	GPIO_Pin_15 // GPIOE

// The value of this definition should be either 1 or -1 only
#define DIR_TOWARDS_B1	-1
#define DIR_TOWARDS_B2	1

void racket_init(void);
void racket_received_command(void);
bool did_receive_command(void);
void racket_update(void);
void racket_calibrate(void);
void open_pneumatic(void);
void close_pneumatic(void);
void serving (void);
u8 get_switch(void);
u8 get_calibrated(void);
u8 get_high_racket_status();

bool get_s1(void);
bool get_s2(void);
bool get_b1(void);
bool get_b2(void);
bool get_b3(void);
s32 get_b1e(void);
s32 get_b2e(void);
s32 get_low_speed(void);
u32 get_it(void);


void add_racket_speed(void);
void decrease_racket_speed(void);
u16 get_racket_speed(void);


#endif
