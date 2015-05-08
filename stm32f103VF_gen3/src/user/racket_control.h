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
#define IR_Sensor_4_Pin GPIO_Pin_4 // C4

#define FOREHAND GPIO_Pin_13
#define UNDERARM GPIO_Pin_12

#define FOREHAND_HOLD_MS	250
#define UNDERARM_HOLD_MS	250

// Sensor functions
void sensor_init(void);
void sensor_update(void);

// Racket functions
void racket_init(void);
void racket_update(void);
void forehand_daa_la(void);
void forehand_lok_la(void);
void underarm_daa_la(void);
void underarm_lok_la(void);

u8  has_forehand_daa_order(void);
u8  has_underarm_daa_order(void);
u32 when_forehand_daa_order(void);
u32 when_underarm_daa_order(void);

#endif
