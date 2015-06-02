#include "racket_control.h"
#include "buzzer_song.h"
#include "delay.h"

static u8   use_sensor = 0;
static u32  sensor_delay = 1500;

static u32  current_time = 0;

static u8   forehand_daa_order = 0;
static u32  forehand_daa_order_time = 0;

static u8   underarm_daa_order = 0;
static u32  underarm_daa_order_time = 0;

void forehand_daa_la(void) {
	if (forehand_daa_order == 0 || get_full_ticks() < forehand_daa_order_time) {
		forehand_daa_order = 1; 
		forehand_daa_order_time = get_full_ticks();
	}
}

void forehand_lok_la(void) {
	if (get_full_ticks() > (forehand_daa_order_time + 100)) {
		forehand_daa_order = 0;
	}
}

void underarm_daa_la(void) {
	if (underarm_daa_order == 0 || get_full_ticks() < underarm_daa_order_time) {
		underarm_daa_order = 1;
		underarm_daa_order_time = get_full_ticks();
	}
}

void underarm_lok_la(void) {
	if (get_full_ticks() > (underarm_daa_order_time + 100)) {
		underarm_daa_order = 0;
	}
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

void sensor_on(void) {
	FAIL_MUSIC;
	use_sensor = 1;
}

void sensor_off(void) {
	FAIL_MUSIC;
	use_sensor = 0;
}

void sensor_increase_delay(void) {
	sensor_delay++;
}

void sensor_decrease_delay(void) {
	if (sensor_delay > 0)
		sensor_delay--;
}

void sensor_update(void) {
	/* if ((us_get_distance(0) > 0 && us_get_distance(0) < 480) || (us_get_distance(1) > 0 && us_get_distance(1) < 480)) {
		// Underarm sensors
		if (use_sensor == 1 && underarm_daa_order == 0) {
			SUCCESSFUL_MUSIC;
			underarm_daa_order = 1; 
			underarm_daa_order_time = get_full_ticks() + 200;
		}
	} */

	if (
		(us_get_distance(2) > 0 && us_get_distance(2) < 823) ||
		(us_get_distance(4) > 0 && us_get_distance(4) < 823)
	) {
		// Forehand sensors
		if (use_sensor == 1 && forehand_daa_order == 0) {
			SUCCESSFUL_MUSIC;
			forehand_daa_order = 1;
			forehand_daa_order_time = get_full_ticks() + (sensor_delay/10);
		}
	}
}

u32 get_sensor_delay(void) {
	return sensor_delay / 10;
}

void racket_init(void) {
	/* GPIO configuration */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Forehand | GPIO_Pin_Underarm;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_WriteBit(GPIOE, GPIO_Pin_Forehand | GPIO_Pin_Underarm, Bit_RESET);
}

void racket_update(void) {
	current_time = get_full_ticks();
	
	if (forehand_daa_order == 1 && current_time >= forehand_daa_order_time) {
		FAIL_MUSIC;
		GPIO_WriteBit(GPIOE, GPIO_Pin_Forehand, Bit_SET);
	} else if (current_time > (forehand_daa_order_time + FOREHAND_HOLD_MS) && forehand_daa_order == 0) {
		GPIO_WriteBit(GPIOE, GPIO_Pin_Forehand, Bit_RESET);
	}
	
	if (underarm_daa_order == 1 && current_time >= underarm_daa_order_time) {
		GPIO_WriteBit(GPIOE, GPIO_Pin_Underarm, Bit_SET);
		FAIL_MUSIC;
	} else if (current_time > (underarm_daa_order_time + UNDERARM_HOLD_MS) && underarm_daa_order == 0) {
		GPIO_WriteBit(GPIOE, GPIO_Pin_Underarm, Bit_RESET);
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
