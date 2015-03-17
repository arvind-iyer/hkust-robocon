#ifndef _XBC_H_
#define _XBC_H_ 


#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "delay.h"


//#define XBC_BUT1	0
// First Byte: xbc_data/down/up[0]
#define XBC_SELECT	0x01
#define XBC_START	0x08
#define XBC_UP		0x10
#define XBC_DOWN	0x40
#define XBC_LEFT	0x80
#define XBC_RIGHT	0x20

//#define XBC_BUT2	1
// Second Byte: xbc_data/down/up[1]
#define XBC_L1	0x04
#define XBC_L2	0x01
#define XBC_R1	0x08
#define XBC_R2	0x02
#define XBC_A	0x10
#define XBC_O	0x20
#define XBC_X	0x40
#define XBC_T	0x80

#define XBC_BUT3	6
// Sixth Byte: xbc_data/down/up[0]
#define XBC_SHIFT	0x01
#define XBC_INC		0x02
#define XBC_DEC		0x04
#define XBC_ALPHA	0x08

// Folling bytes from Analog moded XBC (pscData[2..5] are analog data)
// Left = 0, Middle ~= 0x80, Right = FF
// Top = 0, Middle ~= 0x80, Bottom = FF
#define XBC_JOYRIGHTX	2
#define XBC_JOYRIGHTY	3
#define XBC_JOYLEFTX	4
#define XBC_JOYLEFTY	5

#define XBC_SPI			SPI2
#define XBC_NSS 		GPIO_Pin_12
#define XBC_SPI_PORT 	GPIOB

#define XBC_COMFIRMED	0x00
#define XBC_HELLO		0x01
#define COMMAND_XBC		0x42
#define BIG_CONTROLLER 	0x43
#define	XBOX_CONTROLLER	0x44
#define COMMAND_TFT 	0x52
#define XBC_HANDSHAKE	0x5A
#define XBC_TFT_LINE 	0x80
#define XBC_TFT_COMPLETE 0xF0
#define XBC_TFT_COMPLETE_LINE 0x90

#define XBC_DATA_LENGTH 12

/***********************************************/
/* xbox controller BUT1 & BUT2 digital signal*/
#define XBC_BUT1 	0
#define XBC_BUT2 	1

#define XBC_BUT1_UP		0x01
#define XBC_BUT1_DOWN	0x02
#define XBC_BUT1_LEFT	0x04
#define XBC_BUT1_RIGHT	0x08
#define XBC_BUT1_START	0x10
#define XBC_BUT1_BACK	0x20

#define XBC_BUT2_A		0x10
#define XBC_BUT2_B		0x20
#define XBC_BUT2_X		0x40
#define XBC_BUT2_Y		0x80
#define XBC_BUT2_LB		0x01
#define XBC_BUT2_RB		0x02
#define XBC_BUT2_XBOX	0x04

/* xbox controller analog signal */
//1 byte data
#define XBC_LT		2
#define XBC_RT		3

//2 bytes data
#define XBC_LX_MSB	5
#define XBC_LX_LSB	6

#define XBC_LY_MSB	7
#define XBC_LY_LSB	6

#define XBC_RX_MSB	9
#define XBC_RX_LSB	8

#define XBC_RY_MSB	11
#define XBC_RY_LSB	10
/*******************************************/


void xbc_init(void);
void xbc_gpio_init(void);
void xbc_check_flag (void) ;
void xbc_timer_init(void);
void xbc_spi_consumer (void);
void xbc_tft_transmit(void);
void xbc_chip_select(void);
void xbc_chip_deselect(void);
//void analog_mode_init(void);

extern u8 xbc_mode;
extern u8 volatile old_xbc_mode;
extern u8 volatile xbc_data[XBC_DATA_LENGTH];
extern u8 volatile old_xbc_data[XBC_DATA_LENGTH];
extern u8 volatile xbc_down[3];
extern u8 volatile xbc_up[3];
extern char text[8][16];
extern u8 xbc_done;
extern u8 tft_done;
extern u8 xbc_state;
extern u8 xbc_error_count;

#endif		/*  _XBC_H_ */
