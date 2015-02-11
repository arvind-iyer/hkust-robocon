#ifndef __SERVING_CONTROL_H
#define __SERVING_CONTROL_H

#include "stm32f10x.h"
#include "can_motor.h"
#include "ticks.h"
#include "special_char_handler.h"
#include "stdbool.h"

#define	SWITCH_TIMEOUT					15
#define	PNEUMATIC_OPEN_DELAY	  10
#define MOTOR_OPEN_DELAY        10
#define ENCODER_THRESHOLD				10000

void serving_init(void);
void serving_update(void);
u8 serving_get_switch(void);
s32 serving_get_calibrated(void);
s32 serving_get_current(void);
s32 serving_get_prev(void);

u16 serving_get_racket_speed(void);
u16 serving_get_racket_delay(void);


#endif
