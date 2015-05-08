#ifndef	__SERVING_H
#define	__SERVING_H

#include "stm32f10x.h"
#include "gpio.h"

#include "stm32f10x.h"
#include "ticks.h"
#include "buzzer.h"
#include "gpio.h"
#include "stm32f10x_tim.h"
#include "can_motor.h"

#define	SERVING_SWITCH_GPIO		((GPIO*) (&PE11))

#define	SERVING_VALVE_GPIO		((GPIO*) (&PE10))

#define SERVING_TIM							TIM4
#define SERVING_TIM_RCC					RCC_APB1Periph_TIM4
#define SERVING_IRQn						TIM4_IRQn
#define	SERVING_IRQn_Handler		void TIM4_IRQHandler(void)
//#define	SERVING_SHUTTLECOCK_DROPPING_TIME				(500 * 10)	// 500ms

#define	SERVING_MOTOR						MOTOR5
#define	SERVING_MOTOR_ACC				100

#define	SERVING_UNCALI_SPEED									-200
#define	SERVING_UNCALI_MODE										OPEN_LOOP

#define	SERVING_CALI_SPEED										15
#define	SERVING_CALI_MODE											CLOSE_LOOP
#define	SERVING_CALI_ENCODER_AFTER_SWITCH			-3500
#define	SERVING_CALI_AFTER_SWITCH_SPEED				3
#define	SERVING_CALI_AFTER_SWITCH_MODE				CLOSE_LOOP

#define	SERVING_SHUTTLE_DROP_DELAY_DEFAULT		290
#define	SERVING_HIT_SPEED_DEFAULT							-1500
#define	SERVING_HIT_MODE											OPEN_LOOP
#define	SERVING_HIT_ENCODER_DIFF							24000

#define	SERVING_HIT_STOP_DELAY								800
typedef enum {
	SERVING_CALI_NULL,
	SERVING_CALI_START,
	SERVING_CALI_UNCALI,
	SERVING_CALI_TO_SWITCH,
	SERVING_CALI_SWITCH_PRESSED,
	SERVING_CALI_LOCK
} SERVING_CALI_STATE;

typedef enum {
	SERVING_NULL = 0,
	SERVING_START,
	SERVING_SHUTTLECOCK_DROPPED,
	SERVING_RACKET_START_HITTING,
	SERVING_RACKET_HITTING,
	SERVING_RACKET_STOP_HITTING,
	SERVING_RECALI
} SERVING_HIT_STATE;

void serving_init(void);
void serving_cali_start(void);
void serving_hit_start(void);
void serving_update(void);

s32 get_serving_encoder(void);
s32 get_serving_cali_encoder_target(void);
s32 get_serving_hit_encoder_target(void);


u8 get_serving_calibrated(void);
u8 get_serving_switch(void);
u16 get_shuttle_drop_delay(void);
void set_shuttle_drop_delay(u16 delay_ms);
s16 get_serving_hit_speed(void);
void set_serving_hit_speed(s16 speed); 

SERVING_CALI_STATE get_serving_cali_state(void);
SERVING_HIT_STATE get_serving_hit_state(void);

#endif	/* __SERVING_H */
