#ifndef __SERVE_H
#define __SERVE_H
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

#define SERVE_SWITCH &PE9
#define RACKET MOTOR5
#define SERVE_PNEU0_GPIO &PE15//&PD9
#define	SERVE_PNEU1_GPIO &PE12

#define SERVE_PNEU0_GPIO_BACKUP &PD9
#define	SERVE_PNEU1_GPIO_BACKUP &PB9

#define	SERVE_PNEU_TEST		&PE5
#define	SERVE_SET_COUNT			2

#define ENCODER_THRESHOLD 	-8100 //-10500 //-8500

#define SERVING_TIM							TIM4
#define SERVING_TIM_RCC					RCC_APB1Periph_TIM4
#define SERVING_IRQn						TIM4_IRQn
#define	SERVING_IRQn_Handler		void TIM4_IRQHandler(void)


typedef enum {
	SERVE_END_NULL,
	SERVE_END_TIMEOUT,
	SERVE_END_ENCODER
} SERVE_END;

void serve_timer_init(void);
void serve_update(void);

// interface
void serve_pneu_toggle(void);
void serve_pneu_set(u8 id, u8 flag);
void serve_free(void);
void serve_lock(void);
void serve_calibrate(void);		// THIS IS ALSO THE FUNCTION U CALL
void serve_hit(void);		// DO NOT CALL THIS FUNCTION
void serve_start(u8 id);		// THIS IS FUNCTION U CALL
void fake_serve_start(void);
void start_auto_serve(void);

SERVE_END serve_get_end(void);

// primary control
//void toggle_serve_pneu(void);


// getter and setter for properties
void serve_change_vel(u8 id, s16);
s32 serve_get_vel(u8 id);
void serve_change_delay(u8 id, s16);
u32 serve_get_delay(u8 id);
bool serve_prioritized(void);
void serve_set_vel(u8 id, s16 vel);
void serve_set_delay(u8 id, s16 delay);
s32 serve_get_timeout(void);
void serve_set_timeout(u32);

bool serve_get_failed(void);
bool serve_get_calibrating(void);

#endif
