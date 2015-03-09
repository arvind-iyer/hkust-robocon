#ifndef __ADD_LIB_V0_H
#define __ADD_LIB_V0_H
#include "main.h"
/************************************************************************************/

#define XBC_NSS GPIO_Pin_12 //chip select (CS)
#define XBC_SCK GPIO_Pin_13
#define XBC_MISO GPIO_Pin_14
#define XBC_MOSI GPIO_Pin_15
#define XBC_GPIO GPIOB
#define XBC_SPI SPI2

#define XBC_COMFIRMED	0x81
#define XBC_HELLO		0x01
#define XBC_HANDSHAKE	0x5A
#define XBC_CONTROLLER 	0x46
#define COMMAND_XBC 	0x42
#define COMMAND_TFT_CL	0x51
#define COMMAND_TFT 	0x52
#define FAILURE_DATA    0xFE
#define XBC_TFT_LINE 	0x80

#define read_usb_buff()	ch376_write_cmd(CH376_CMD_RD_USB_DATA0);


void spi_init(void);

void usb_read_data_buf(void);
void xbx_check_flag(void);
void xbc_chip_deselect(void);
s8 get_line_number(void);
void reset_line_number(void);
u8 reset_spi_transmit_flag(void);

extern u8 xbc_data[20];
extern u8 tx_xbc_data[20];
extern u8 read_data_err[20];
extern u8 is_failure_data;
extern u8 data_busy;
extern u8 tft_receive_flag;
#endif	/* __ADD_LIB_V0_H */		

