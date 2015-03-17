#ifndef CH376_DRIVER_H
#define CH376_DRIVER_H
/*
#define STM32						0
#define ATMEGA128					1
#define MCU 						STM32

#define DELAY_NORMAL				5
#define DELAY_BAUDRATE				1
#define DELAY_SET_USB_MODE			15
#define DELAY_100US					100

#define DELAY_HW_RESET_PIN_HIGH		10
#define DELAY_HW_RESET_WAIT			10
#define DELAY_SW_RESET_WAIT			40

#define DUMMY_PACKET				0xFF


#if MCU == ATMEGA128

#define SPI_SEND(x)				
#define SPI_RECIEVE()			
#define SPI_IS_BUSY()			

#define SET_CS()				
#define CLR_CS()				

#define SET_RST()				
#define CLR_RST()				

#define HV_INTERRUPT()			
	
#else
*/
#include "main.h"
#include "system.h"
#include "stm32f10x_tim.h"


/* config */
/*
#define 	XBC_CTL_WR   	GPIO_Pin_7	//GPIOB
#define  	XBC_CTL_RD		GPIO_Pin_8 //GPIOB
#define 	XBC_CTL_PCS		GPIO_Pin_9	//GPIOB
#define 	XBC_CTL_AO 	 	GPIO_Pin_1	//GPIOG
#define     CH376_INT_N		GPIO_Pin_1  //GPIOE
#define     CH376_HW_RST	GPIO_Pin_0  //GPIOE
*/

#define 	XBC_CTL_AO 	 	GPIO_Pin_5	//GPIOB
#define  	XBC_CTL_RD		GPIO_Pin_6  //GPIOB
#define 	XBC_CTL_WR   	GPIO_Pin_7	//GPIOB
#define 	XBC_CTL_PCS		GPIO_Pin_8  //GPIOB

#define     CH376_HW_RST	GPIO_Pin_2  //GPIOA
#define     CH376_INT_N		GPIO_Pin_3  //GPIOA

#define 	BUT1			GPIO_Pin_6	//C
#define 	BUT2			GPIO_Pin_8  //C
#define 	BUT3			GPIO_Pin_9  //C
#define 	BUT4			GPIO_Pin_12 //A
#define 	BUT5			GPIO_Pin_12 //C
#define 	BUT6			GPIO_Pin_7  //C

#define P0		GPIO_Pin_6	//PC   <-- sw reset keys
#define P1 		GPIO_Pin_9	//PC
#define P7		GPIO_Pin_8	//PC
#define P2		GPIO_Pin_12  //PA  <-- sw reset keys
#define P5		GPIO_Pin_12	//PC
#define P6		GPIO_Pin_7	//PC


/*GPIOF data IO*/
/*
#define 	XBC_DATA_PORT	GPIOF
#define		XBC_DATA_D0	 	GPIO_Pin_1 
#define 	XBC_DATA_D1 	GPIO_Pin_2	
#define 	XBC_DATA_D2 	GPIO_Pin_3	
#define		XBC_DATA_D3   	GPIO_Pin_4  
#define 	XBC_DATA_D4 	GPIO_Pin_5    
#define 	XBC_DATA_D5  	GPIO_Pin_13	
#define  	XBC_DATA_D6  	GPIO_Pin_14   
#define  	XBC_DATA_D7  	GPIO_Pin_15   
*/
#define 	XBC_DATA_PORT_0_1	GPIOB
#define		XBC_DATA_D0	 	GPIO_Pin_1
#define 	XBC_DATA_D1 	GPIO_Pin_0

#define 	XBC_DATA_PORT_2_7	GPIOC	
#define 	XBC_DATA_D2 	GPIO_Pin_5	
#define		XBC_DATA_D3   	GPIO_Pin_4  
#define 	XBC_DATA_D4 	GPIO_Pin_3    
#define 	XBC_DATA_D5  	GPIO_Pin_2
#define  	XBC_DATA_D6  	GPIO_Pin_1 
#define  	XBC_DATA_D7  	GPIO_Pin_0  


#define DELAY_MS_FUNCTION(x)	_delay_ms(x)
#define DELAY_US_FUNCTION(x)	_delay_us(x)

#define HV_INTERRUPT()			((GPIO_ReadInputDataBit(GPIOA, CH376_INT_N))? 0 : 1	)
#define HV_INT					GPIO_ReadInputDataBit(GPIOA, CH376_INT_N)		
/*
#endif

#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE   0
#endif
*/
//  USB Mode For ch376 Set Mode Use
#define CH376_MODE_INVALID_DEV		0x00
#define CH376_MODE_DEV_PF			0x01
#define CH376_MODE_DEV_IF			0x02
#define CH376_MODE_SD_HOST			0x03
#define CH376_MODE_INVALID_HOST		0x04
#define CH376_MODE_HOST_NON_SOF		0x05
#define CH376_MODE_HOST_SOF			0x06
#define CH376_MODE_HOST_RST			0x07

//	USB Speed For ch376 Set USB Speed Use
#define CH376_USB_12M_FULL_SPEED	0x00
#define CH376_USB_1_5M_FULL_SPEED	0x01
#define CH376_USB_1_5M_LOW_SPEED	0x02

//	Descriptor Type For ch376 Get DESCR Use
#define CH376_DESCR_DEVICE			0x01
#define CH376_DESCR_CONFIGURATION	0x02

//	Token PID For ch376 Issue TKN X Use
#define CH376_TKN_SETUP				0x0D
#define CH376_TKN_OUT				0x01
#define CH376_TKN_IN				0x09

//	Data phase For ch376 Issue TKN X Use
#define CH376_TKN_PHASE_DATA0		0x00
#define CH376_TKN_PHASE_DATA1		0x80


//	Endpoint Phase For ch376 Set Endpoint Use
#define CH376_ENDP_DATA0			0x80
#define CH376_ENDP_DATA1			0xC0

//	ch376 Control Status
#define	CH376_CTRL_STATUS_SUCCESS	0x51
#define	CH376_CTRL_STATUS_ABORT		0x5F	

//	ch376 Interrupt Status
#define CH376_USB_INT_SUCCESS		0x14
#define CH376_USB_INT_CONNECT		0x15
#define CH376_USB_INT_DISCONNECT	0x16
#define CH376_USB_INT_BUF_OVER		0x17
#define CH376_USB_INT_USB_READY		0x18
#define CH376_USB_INT_DISK_READ		0x1D
#define CH376_USB_INT_DISK_WRITE	0x1E
#define CH376_USB_INT_DISK_ERR		0x1F

#define CH376_USB_INT_PING			0x24
#define CH376_USB_INT_SPLIT			0x28
#define CH376_USB_INT_NAK			0x2A
#define CH376_USB_INT_PRE			0x2C
#define CH376_USB_INT_STALL			0x2E

//	ch376 Command
#define CH376_CMD_GET_IC_VER		0x01
#define CH376_CMD_SET_BAUDRATE		0x02
#define CH376_CMD_ENTER_SLEEP		0x03
#define CH376_CMD_RESET_ALL			0x05
#define CH376_CMD_CHECK_EXIST		0x06
#define CH376_CMD_SET_SD0_INT		0x0B
#define CH376_CMD_GET_FILE_SIZE		0x0C
#define CH376_CMD_SET_USB_MODE		0x15
#define CH376_CMD_GET_STATUS		0x22
#define CH376_CMD_RD_USB_DATA0		0x27
#define CH376_CMD_WR_HOST_DATA		0x2C
#define CH376_CMD_WR_REQ_DATA		0x2D
#define CH376_CMD_WR_OFS_DATA		0x2E
#define CH376_CMD_SET_FILE_NAME		0x2F
#define CH376_CMD_DISK_CONNECT		0x30
#define CH376_CMD_DISK_MOUNT		0x31
#define CH376_CMD_DIR_INFO_READ		0x37
#define CH376_CMD_DIR_INFO_SAVE		0x38
#define CH376_CMD_BYTE_LOCATE		0x39
#define CH376_CMD_BYTE_READ			0x3A
#define CH376_CMD_BYTE_RD_GO		0x3B
#define CH376_CMD_BYTE_WRITE		0x3C
#define CH376_CMD_BYTE_WR_GO		0x3D
#define CH376_CMD_SEC_LOCATE		0x4A
#define CH376_CMD_SEC_READ			0x4B
#define CH376_CMD_SEC_WRITE			0x4C
#define CH376_CMD_SET_USB_SPEED		0x04
#define CH376_CMD_GET_DEV_RATE		0x0A
#define CH376_CMD_READ_VAR8			0x0A
#define CH376_CMD_SET_RETRY			0x0B
#define CH376_CMD_WRITE_VAR8		0x0B
#define CH376_CMD_READ_VAR32		0x0C
#define CH376_CMD_WRITE_VAR32		0x0D
#define CH376_CMD_DELAY_100US		0x0F
#define CH376_CMD_SET_USB_ADDR		0x13
#define CH376_CMD_TEST_CONNECT		0x16
#define CH376_CMD_ABORT_NAK			0x17
#define CH376_CMD_SET_ENDP6			0x1C
#define CH376_CMD_SET_ENDP7			0x1D
#define CH376_CMD_DIRTY_BUFFER		0x25
#define CH376_CMD_CLR_STALL			0x41
#define CH376_CMD_SET_ADDRESS		0x45
#define CH376_CMD_GET_DESCR			0x46
#define CH376_CMD_SET_CONFIG		0x49
#define CH376_CMD_AUTO_SETUP		0x4D
#define CH376_CMD_DISK_MAX_LUN		0x5D
#define CH376_CMD_ISSUE_TKN_X		0x4E
#define CH376_CMD_EP1_USB_PID_IN	0x19 //end point 1 
#define CH376_CMD_EP0_USB_PID_IN	0x09 

//	ch376 Command Data
#define CH376_DATA0_SET_SD0_INT		0x16
#define CH376_DATA1_SET_SD0_INT_ON	0x90
#define CH376_DATA1_SET_SD0_INT_OFF	0x10

// issue tkn x error handling 
#define	CH376_ISSUE_DATA_PID_EVEN	0x23
#define	CH376_ISSUE_DATA_PID_ODD	0x2B
#define CH376_ISSUE_NAK_DATA		0x00
#define CH376_ISSUE_USB_READY		0x18
#define PID_EVEN_MASK				0x80
#define PID_ODD_MASK				0x7F

#define CH376_DATA0_GET_DEV_RATE	0x07

#define CH376_DATA0_SET_RETRY		0x25
//init command
void reset_all_pin(void);
void ch376_hardware_init(void);
void ch376_hardware_reset(void);
void ch376_software_reset(void);

//I/O command
void ch376_write_cmd(u8 cmd);
void ch376_write_data(u8 data);
void ch376_write_cmd_data(u8 cmd, u8 data);
void ch376_write_cmd_data2(u8 cmd, u8 data0, u8 data1);
u8 read_data_pp(void);
u8 ch376_read_data(void);
u8 ch376_read_cmd_data(u8 cmd, u8 data);
void ch376_no_operation(void);

//set usb mode
void ch376_set_usb_speed (u8 speed);
u8 ch376_set_usb_mode(u8 mode);

//set spi mode(useless in this program)
void ch376_set_sd0_init(void);

u8 usb_to_ch376_read(void);

//ch376 functions 
u8 ch376_test_connect(void);
void ch376_set_retry(u8 times);
u8 ch376_get_int_status(void);
u8 ch376_check_exist(u8 data);

void write_data_to_usb(u8 len, u8 data[]);

void ns_delay(void);

#endif
