#ifndef _XBC_H_
#define _XBC_H_ 


#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "delay.h"
#include "tft.h"
#include "ticks.h"

#define XBC_SPI			SPI2
#define XBC_NSS 		GPIO_Pin_12
#define XBC_SPI_PORT 	GPIOB

#define XBC_SPI_SCK		GPIO_Pin_13
#define XBC_SPI_MISO	GPIO_Pin_15
#define XBC_SPI_MOSI	GPIO_Pin_14
#define XBC_SPI_NSS		GPIO_Pin_12

#define XBC_COMFIRMED	0x81
#define XBC_HELLO		0x01
#define	XBC_BOARD_BUSY	0x10
#define COMMAND_XBC		0x42
//#define BIG_CONTROLLER	0x43
#define	XBOX_CONTROLLER	0x46
#define COMMAND_TFT_CL	0x51
#define COMMAND_TFT 	0x52
#define XBC_HANDSHAKE	0x5A
#define FAILURE_DATA    0xFE
#define XBC_TFT_LINE 	0x80
#define XBC_TFT_COMPLETE 0xF0
#define XBC_TFT_COMPLETE_LINE 0x90
#define XBC_DATA_LENGTH 20

/***********************************************/
/* 
 * xbox controller digital signal 
 * 16 bit form 	
 */
#define XBC_UP		0x0001
#define XBC_DOWN	0x0002
#define XBC_LEFT	0x0004
#define XBC_RIGHT	0x0008
#define XBC_START	0x0010
#define XBC_BACK	0x0020
#define XBC_L_JOY	0x0080
#define XBC_R_JOY	0x0040

#define XBC_A		0x1000
#define XBC_B		0x2000
#define XBC_X		0x4000
#define XBC_Y		0x8000
#define XBC_LB		0x0100
#define XBC_RB		0x0200
#define XBC_XBOX	0x0400

//bottom buttons
#define L_BUT1	0x010000
#define L_BUT2	0x020000
#define L_BUT3	0x040000
#define R_BUT1	0x080000
#define R_BUT2	0x100000
#define R_BUT3	0x200000

/* xbox controller analog signal */
//1 byte data
#define XBC_LT		0
#define XBC_RT		1

//2 bytes data
#define XBC_LX		2
#define XBC_LY     	3
#define XBC_RX  	4
#define XBC_RY  	5


/* variable */
#define DISCONNECTED 0
#define CONNECTED	 1

#define LCD_EXTEND_ON	1
#define LCD_EXTEND_OFF	0

/*******************************************/
/*public usage: xbox controller data*/
extern u32 volatile xbc_digital; //conbine two 8bits variable data into one 16bits
extern s16 volatile xbc_joy[6];  
extern volatile u32 xbc_press; // trigger after button pressed
extern volatile u32 xbc_release; // trigger after button released
extern u8 volatile xbc_mode; // 0 disconnect, 1 connect
extern u8 running;
u8 xbc_update(void);
/*******************************************/

/************/
//test
extern u16 disconnect_cnt;

/************************/


void xbc_init(u8);
void xbc_gpio_init(void);

void xbc_timer_init(void);
void xbc_spi_consumer (void);
void xbc_tft_transmit(void);

extern u8 volatile xbc_data[XBC_DATA_LENGTH];
//extern char text[8][16];
extern u8 xbc_done;
extern u8 tft_done;
extern u8 xbc_state;

void disable_xbc(void);
void enable_xbc(void);
u8 get_xbc_mode(void);
void xbc_test_program(void);


#endif		/*  _XBC_H_ */
