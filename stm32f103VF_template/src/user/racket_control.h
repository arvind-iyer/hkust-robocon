#ifndef __RACKET_CONTROL_H
#define __RACKET_CONTROL_H

#include "stm32f10x.h"
#include "can_motor.h"
#include "ticks.h"
#include "special_char_handler.h"
#include "stdbool.h"

//#define	RACKET_TIMEOUT					25
#define	PNEUMATIC_OPEN_DELAY	  10
#define MOTOR_OPEN_DELAY        10

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

#endif
