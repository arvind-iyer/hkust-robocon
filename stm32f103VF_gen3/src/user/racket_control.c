#include "racket_control.h"
#include "buzzer_song.h"
#include "delay.h"

static u32  current_time = 0;
static bool stop_switch = 0;
static bool low_switch = 0;
static bool high_switch = 0;
static bool stop_flag = 0;

// Lower racket variables
static u8   forehand_daa = 0;
static u8   forehand_daa_order = 0;
static u32  forehand_daa_order_time = 0;

static u8   underarm_daa = 0;
static u8   underarm_daa_order = 0;
static u32  underarm_daa_order_time = 0;

void forehand_daa_la(void) {
	if (forehand_daa_order == 0) {
		forehand_daa_order = 1; 
		forehand_daa_order_time = get_full_ticks();
	}
}

void forehand_lok_la(void) {
	forehand_daa_order = 0;
}

void underarm_daa_la(void) {
	if (underarm_daa_order == 0) {
		underarm_daa_order = 1;
		underarm_daa_order_time = get_full_ticks();
	}
}

void underarm_lok_la(void) {
	underarm_daa_order = 0;
}

void sensor_init(void) {/*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = IR_Sensor_1_Pin | IR_Sensor_2_Pin | IR_Sensor_3_Pin;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = IR_Sensor_4_Pin;
	GPIO_Init(GPIOC, &GPIO_InitStructure);*/
}

void racket_init(void) {
	/* GPIO configuration */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = FOREHAND | UNDERARM;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOE, FOREHAND);
	GPIO_SetBits(GPIOE, UNDERARM);
}

void sensor_update(void) { /*
	if (request_low_racket_move == 0 && use_high_switch == 0) {
		if (GPIO_ReadInputDataBit(GPIOA, IR_Sensor_1_Pin)) {
		// High
				low_racket_move();
				SUCCESSFUL_MUSIC;
		} else if (GPIO_ReadInputDataBit(GPIOA, IR_Sensor_2_Pin)) {
		// Mid
				low_racket_move();
				SUCCESSFUL_MUSIC;
		} else if (GPIO_ReadInputDataBit(GPIOA, IR_Sensor_3_Pin)) {
		// Low
				low_racket_move();
				SUCCESSFUL_MUSIC;
		} else if (GPIO_ReadInputDataBit(GPIOC, IR_Sensor_4_Pin)) {
			low_racket_move();
			SUCCESSFUL_MUSIC;
		}
	} */
	
}

void racket_update(void) {
	current_time = get_full_ticks();
	
	if (forehand_daa_order == 1) {
		GPIO_ResetBits(GPIOE, FOREHAND);
	} else if (forehand_daa_order_time - current_time > FOREHAND_HOLD_MS && forehand_daa_order == 0) {
		GPIO_SetBits(GPIOE, FOREHAND);
	}
	
	if (underarm_daa_order == 1) {
		GPIO_ResetBits(GPIOE, UNDERARM);
	} else if (underarm_daa_order_time - current_time > UNDERARM_HOLD_MS && underarm_daa_order == 0) {
		GPIO_SetBits(GPIOE, UNDERARM);
	}
	
}


u8 has_forehand_daa_order(void) {
	return forehand_daa_order;
}

u8 has_underarm_daa_order(void) {
	return underarm_daa_order;
}

u32 when_forehand_daa_order(void) {
	return forehand_daa_order_time;
}

u32 when_underarm_daa_order(void) {
	return underarm_daa_order_time;
}