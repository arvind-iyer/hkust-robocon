#ifndef __RACKET_H
#define __RACKET_H
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "special_char_handler.h"
#include "button.h"
#include "can_motor.h"
#include "approx_math.h"
#include "servo.h"
#include "gpio.h"
#include "serve.h"
#include "robocon.h"

//#define RACKET MOTOR5

//#define RACKET_SWITCH_LINE 	EXTI_Line2
//#define RACKET_IRQn 			 	EXTI2_IRQn
//#define RACKET_PIN_SOURCE 	GPIO_PinSource2


//#define ROTATE_SWITCH_LINE 	EXTI_Line3
//#define ROTATE_IRQn 			 	EXTI3_IRQn
//#define ROTATE_PIN_SOURCE 	GPIO_PinSource3

//#define RACKET_SWITCH_INTERRUPT_HANDLER void EXTI2_IRQHandler(void)
//#define ROTATE_SWITCH_INTERRUPT_HANDLER void EXTI3_IRQHandler(void)	
	
//#define ENCODER_THRESHOLD 	12500

//Controls
//#define KEY_STOP_RACKET 'p'
//#define KEY_LOCK_RACKET 'o'
//#define KEY_CALIB_RACKET 'l'
//#define KEY_HIT_RACKET 'k'

#define LASER_GPIO &PE7
#define PNEU_GPIO ROBOT=='C'?&PD9:&PD10
#define PNEU_GPIO_DOWN &PD10

static u16 RACKET_HIT_DELAY_TIME = 100;
//void racket_init(void);


void racket_update(void);

void racket_delayed_hit(int delay);
void racket_down_hit(void);
 void racket_hit(void);
void racket_pneumatic_set(bool);
void racket_pneumatic_2_set(bool);

#endif
