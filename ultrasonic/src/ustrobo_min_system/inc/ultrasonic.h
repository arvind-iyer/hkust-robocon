#ifndef __US_H
#define __US_H

#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "gpio.h"

#define US_TIM							TIM10
#define US_RCC							RCC_APB2Periph_TIM10
#define US_IRQn						  TIM1_UP_TIM10_IRQn
#define US_IRQHandler			  void TIM1_UP_TIM10_IRQHandler(void)


#define US_TRIG_PULSE         10    // 10 us
#define US_ECHO_PULSE_COUNT   5
#define US_MAX_WIDTH					1000	// 3400mm

#define	US_IDLE_RESET_COUNT		1000
#define	US_TAKE_TURN_BREAK		100

#define	US_DEVICE_COUNT				3

#define	US_CAN_ID							0x90


typedef enum {
	US_NULL								= 0,
	US_READY							= 1,
	US_TRIGGER 						= 2,
	US_WAITING_FOR_ECHO		= 3,
	US_ECHO_RAISED				= 4,
	US_ECHO_FALLEN				= 5
	
} US_STATE;

typedef struct {
	const GPIO* trig_gpio, *echo_gpio;
	US_STATE state;
	u32 pulse_width_tmp, pulse_width;
	u32 idle_width;
	u8 rx_complete;	// For sync
	u16 last_sample_second;
	u16 last_sample_count;
	u16 sample_count;
} US_TypeDef;

typedef enum {
	US_INDEPENDENT,
	US_SYNC,
	US_TAKE_TURN
} US_MODE;


void us_init(US_MODE mode);
US_MODE us_get_mode(void);
US_STATE us_get_state(u8 i);
u32 us_get_pulse_raw(u8 i);
u32 us_get_pulse(u8 i);
u32 us_get_distance(u8 i);
u16 us_get_speed(u8 i);
u8 us_get_current_us(void);

#endif /* __US_H */
