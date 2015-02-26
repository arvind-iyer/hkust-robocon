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
#include "robocon.h"

#define RACKET MOTOR5

#define RACKET_SWITCH_LINE 	EXTI_Line2
#define RACKET_IRQn 			 	EXTI2_IRQn
#define RACKET_PIN_SOURCE 	GPIO_PinSource2


#define ROTATE_SWITCH_LINE 	EXTI_Line3
#define ROTATE_IRQn 			 	EXTI3_IRQn
#define ROTATE_PIN_SOURCE 	GPIO_PinSource3

#define RACKET_SWITCH_INTERRUPT_HANDLER void EXTI2_IRQHandler(void)
#define ROTATE_SWITCH_INTERRUPT_HANDLER void EXTI3_IRQHandler(void)	
	
#define ENCODER_THRESHOLD 	7200

//Controls
#define KEY_STOP_RACKET 'p'
#define KEY_LOCK_RACKET 'o'
#define KEY_CALIB_RACKET 'l'
#define KEY_HIT_RACKET 'k'




//void racket_init(void);
void racket_update(void);
 void racket_stop(void);
 void racket_lock(void);
 void racket_calibrate(void);
 void racket_hit(void);
void racket_start_serve(void);
 void racket_set_vel(s32 vel, CLOSE_LOOP_FLAG loop);
 s32 racket_get_vel(void);
 void racket_hit_on(void);
void racket_hit_off(void);
void racket_increase_hit_vel(void);
void racket_decrease_hit_vel(void);
void racket_serve_increase_delay(void);
void racket_serve_decrease_delay(void);
u32 racket_get_serve_delay(void);
s32 get_init_enc(void);
void toggle_servo(void);
s32 racket_get_last_stop_encoder_value();
#endif
