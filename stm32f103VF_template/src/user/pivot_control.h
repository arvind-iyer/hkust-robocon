#ifndef __PIVOT_CONTROL_H
#define __PIVOT_CONTROL_H

#include "stm32f10x.h"
#include "can_motor.h"
#include "ticks.h"
#include "special_char_handler.h"
#include "stdbool.h"

#define SWITCH_GPIO GPIOE    //GPIO?
#define SWITCH_LEFT_Pin GPIO_Pin_1   //Pin? 
#define SWITCH_RIGHT_Pin GPIO_Pin_2    //Pin?

#define Motor_delay 20


void pivot_init(void);
void pivot_update(void);
void pivot_turn_left(void);
void pivot_turn_right(void);
u8 get_left_mode(void);
u8 get_right_mode(void);
u8 get_motor_mode(void);

#endif
