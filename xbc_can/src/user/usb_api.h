#ifndef USB_API_H
#define USB_API_H

#include "ch376_driver.h"
//#include "usb_4_byte_mouse.h"

#define USB_STATE_NOT_INIT		0x00
#define USB_STATE_DISCONNECTING	0x01
#define USB_STATE_DISCONNECTED	0x02
#define USB_STATE_ATTACHED		0x03
#define USB_STATE_RESETED		0x04
#define USB_STATE_DEFAULT		0x05
#define USB_STATE_AUTOSET		0x06
#define USB_STATE_SETSPEED		0x07
#define USB_STATE_TKN_X			0x08
#define	USB_STATE_CONFIGURED	0x09
#define USB_STATE_SUSPENDED		0xF0
#define USB_STATE_ERROR			0xFF

#define CHECK_EXIST_DATA			0x5A
//#define EXIST_DATA					0xA5

#define USB_CHECK_TIME			20
#define UPDATE_TIME 			20

/**
 *	Retry Times	
 *	0b **xx xxxx			Retry 0b00xx xxxx Times if times out
 *	0b 10** ****			Retry unlimited times if NAK is received
 *	0b 11** ****			Retry maximum for 3 second if NAK is received
 *	0b 0*** ****			Report Error if NAK is received
**/

#define USB_RETRY_TIMES			0xE0

#define USB_12M_FULL_SPEED		CH376_USB_12M_FULL_SPEED
#define USB_1_5M_FULL_SPEED		CH376_USB_1_5M_FULL_SPEED
#define USB_1_5M_LOW_SPEED		CH376_USB_1_5M_LOW_SPEED

///	Mask for changing the phasing
#define USB_PHASING_MASK		0x80
#define CHECK_USB_BUFF(x)		ch376_write_cmd_data2(CH376_CMD_ISSUE_TKN_X, x, CH376_CMD_EP1_USB_PID_IN);
#define DEF_USB_BUFF(x)			ch376_write_cmd_data2(CH376_CMD_ISSUE_TKN_X, x, CH376_TKN_SETUP);


void usb_init(void);
void reset_xbc(u8 i);
u8 get_xbc_id(void);
u8 usb_main_loop(void);
u8 check_ch376_usb_conn(void);
u8 usb_get_state(void);
u8 usb_connected(void);

void usb_start_run(void);
void usb_stop_run(void);

void usb_set_speed(u8 speed);
void usb_set_retry_times(u8 times);


u8   usb_disconnect(void);
u8   usb_reset(void);
u8   usb_connect(void);
u8   usb_test_connection(void);
u8   usb_hv_interrupt(void);
u8   usb_get_status(void);
u8   usb_check_exist(void);
void usb_auto_setup(void);
u8 	 check_usb_data_buff(void);

u8 read_tkn_status(void);
u8 get_usb_state(void);
void tracking_xbc(void);
void xbc_config (u8 function);
u8 set_xbc_config(u8 function);// 1: setting success, 0 not ok yet
//extern u8 is_usb_ready;

#endif
