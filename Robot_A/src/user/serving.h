#ifndef	__SERVING_H
#define	__SERVING_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "gpio.h"

#include "stm32f10x.h"
#include "ticks.h"
#include "buzzer.h"
#include "gpio.h"
#include "stm32f10x_tim.h"
#include "can_motor.h"
#include "approx_math.h"


#define	SERVING_SWITCH_GPIO		((GPIO*) (&PE11))

#define	SERVING_VALVE_GPIO		((GPIO*) (&PB9))

#define SERVING_TIM							TIM4
#define SERVING_TIM_RCC					RCC_APB1Periph_TIM4
#define SERVING_IRQn						TIM4_IRQn
#define	SERVING_IRQn_Handler		void TIM4_IRQHandler(void)
//#define	SERVING_SHUTTLECOCK_DROPPING_TIME				(500 * 10)	// 500ms

#define	SERVING_SET_COUNT											2			// 2 sets of serving

#define	SERVING_MOTOR													MOTOR5
#define	SERVING_MOTOR_ACC											150
#define	SERVING_MOTOR_ENCODER_CYCLE						28000				

#define	SERVING_REPRESS_SPEED									200
#define	SERVING_REPRESS_MODE									OPEN_LOOP

#define	SERVING_UNCALI_SPEED									20							/*!< Uncalibration motor speed */
#define	SERVING_UNCALI_MODE										CLOSE_LOOP				/*!< Uncalibration motor mode */
#define	SERVING_UNCALI_SWITCH_OFF_COUNT				30								/*!< Count of the switch off, to prevent bouncing */

#define	SERVING_CALI_SPEED										-8							/*!< Calibration motor speed (when switch is off) */
#define	SERVING_CALI_MODE											CLOSE_LOOP			/*!< Calibration motor mode  (when switch is off)*/
#define	SERVING_CALI_SWITCH_ON_COUNT					100							/*!< Count of the switch on, to prevent bouncing */
#define	SERVING_CALI_ENCODER_AFTER_SWITCH			1200						/*!< Calibration motor speed (when switch is on) */
#define	SERVING_CALI_AFTER_SWITCH_SPEED				-3							/*!< Calibration motor mode  (when switch is on) */
#define	SERVING_CALI_AFTER_SWITCH_MODE				CLOSE_LOOP			/*!< Calibration motor mode  (when switch is on) */
#define	SERVING_CALI_LOCK_TIME_MS							200							/*!< Calibration lock time (in ms), just for can tx*/
#define	SERVING_CALI_TIMEOUT									4000						/*!< Stop calibrating after the timeout (ms) */


#define	SERVING_SHUTTLE_DROP_DELAY_DEFAULT		340							/*!< Default value of the shuttle drop delay in ms */
#define	SERVING_HIT_SPEED_DEFAULT							-1700						/*!< Default motor speed for hitting */
#define	SERVING_HIT_MODE											OPEN_LOOP				/*!< Motor mode for hitting */
#define	SERVING_HIT_ENCODER_DIFF							15000						/*!< The encoder value diff, that the hitting will stop */
#define	SERVING_HIT_TIMEOUT										800						/*!< Stop serving after the timeout (ms) */

#define	SERVING_HIT_STOP_DELAY								600							/*!< The delay (in ms) after hitting */

typedef enum {
	SERVING_CALI_NULL,								/*!< No calibration is ongoing */
	SERVING_CALI_START,								/*!< Calibration starts */
	SERVING_CALI_REPRESS_SWITCH,			/*!< Re-press the button if the encoder value is in the other half cycle */
	SERVING_CALI_UNCALI,							/*!< Uncalibrate if it was calibrated or the switch is pressed */
	SERVING_CALI_TO_SWITCH,						/*!< Motor moves until the switch is on */
	SERVING_CALI_SWITCH_PRESSED,			/*!< The switch is detected as on */
	SERVING_CALI_LOCK									/*!< The motor is locked */
} SERVING_CALI_STATE;

typedef enum {
	SERVING_NULL = 0,									/*!< No serving is ongoing */
	SERVING_START,										/*!< Serving starts */
	SERVING_SHUTTLECOCK_DROPPED,			/*!< Shuttlecock is dropped */
	SERVING_RACKET_START_HITTING,			/*!< The motor starts moving */
	SERVING_RACKET_HITTING,						/*!< The motor is moving and waiting for the termination case */
	SERVING_RACKET_STOP_HITTING,			/*!< Termination case fulfilled, motor stop moving */
	SERVING_RECALI										/*!< Recalibration */
} SERVING_HIT_STATE;

void serving_init(void);
bool serving_cali_start(void);
bool serving_hit_start(void);
void serving_update(void);

s32 get_serving_encoder(void);
s32 get_serving_cali_encoder_target(void);
s32 get_serving_hit_encoder_target(void);

u8 get_serving_set(void);
void set_serving_set(u8 i);

bool get_serving_calibrated(void);
void serving_uncalibrate(void);
u8 get_serving_switch(void);
u16 get_shuttle_drop_delay(void);
void set_shuttle_drop_delay(u16 delay_ms);
s16 get_serving_hit_speed(void);
void set_serving_hit_speed(s16 speed); 

SERVING_CALI_STATE get_serving_cali_state(void);
SERVING_HIT_STATE get_serving_hit_state(void);

#endif	/* __SERVING_H */
