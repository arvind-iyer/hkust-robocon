#ifndef __MAIN_H
#define __MAIN_H

#include <stdio.h>
#include "stm32f10x.h"	  
#include "stm32f10x_it.h"
#include "delay.h"
#include "ticks.h"
#include "uart.h"
//#include "debug.h"
#include "system.h"
//#include "tft_160x128.h"
#include "lcd_red.h"
#include "ch376_driver.h"
#include "usb_api.h"
#include "xbc.h"

#define FALSE 0
#define TRUE 1
#define NULL 0



//extern u16 ticks_img;
//extern u16 seconds_img ;
extern u8 volatile is_xbc_board_processing;
extern u8 volatile is_spi_interrupting;
void init_xbc_board(void);
void main_read_xbc_data(void);

extern u8 volatile test_exti;
extern u8 volatile finish_init;

#endif /* __MAIN_H */
