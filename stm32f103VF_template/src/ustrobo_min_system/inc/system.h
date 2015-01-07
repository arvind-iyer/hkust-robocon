#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include <string.h>
#include "main.h"

#define COMPUTER_NAME "COMPUTER_NAME"

#define	MENU_MAX 32

void menu_start(char title[50], u16 cnt);
void menu(u8 index0);
void menu_add(u8 i, char title[32], void (*func)(void));

void blank(void);
void encoder_test(void);
void position_test(void);
void servo_test(void);
void pwm_test(void);
void get_program_info(void);
void gpioe_test(void);
void battery_low(void);

#endif	/* __SYSTEM_H */	
