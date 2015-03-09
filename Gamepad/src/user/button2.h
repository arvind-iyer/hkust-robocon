#ifndef __BUTTON_H
#define __BUTTON_H

#define MAINBOARD_V2

#include "stm32f10x.h"
#include "delay.h"
//#include "psc.h"

#define UP 1
#define DOWN 2
#define LEFT 4
#define RIGHT 8
#define START 16
#define WAKEUP 32

 
#ifdef MAINBOARD_V2		//for mainboard v2
#define BUT_UP		GPIO_Pin_7
#define BUT_DOWN	GPIO_Pin_8
#define BUT_START		GPIO_Pin_6
#define BUT_L	    GPIO_Pin_9
#define BUT_R	    GPIO_Pin_10


#else					// for mainboard v1 
#define BUT_UP		GPIO_Pin_6
#define BUT_DOWN	GPIO_Pin_7
#define BUT_START	GPIO_Pin_8
#define BUT_L	    GPIO_Pin_9
#define BUT_R	    GPIO_Pin_10
*/
#endif

//#define BUT_WAKEUP  GPIO_Pin_0		 //use GPIOA

#define BUT_PORT	GPIOF
#define BUT_RCC		RCC_APB2Periph_GPIOF


extern u8 button_down;
extern u8 button_data;
extern u8 tft_orientation;
void button_init(void);
void button_update(void); 		
extern u8 _BV(u8);

#endif /* __BUTTON_H */

