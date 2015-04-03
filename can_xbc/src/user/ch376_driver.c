#include "ch376_driver.h"


u8 GET_DESCR_CMD[8] = { 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00 };
u8 SET_DESCR_CMD[8] = { 0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x04, 0x00 };
u8 SET_USB_ADDR_CMD[8] = { 0x00, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 SET_USB_CONFIG_CMD[8] = { 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

//u8 read_data_buf[65] = {0}; 
u16 Data=0;
u8 i = 0;
u8 descr_data1 = 0, descr_data2 = 0; //DESCRIPTOR

#define RESET_ALL 0x05
#define SET_USB_MODE 0x15
#define GET_STATUS 0x22	
#define RD_USB_DATA0 0x27
#define RD_USB_DATA 0x28
#define WR_HOST_DATA 0x2c
#define WR_REQ_DATA 0x2d
#define	WR_USB_DATA5 0x2a	
#define	WR_USB_DATA7 0x2b
#define GET_DESCR 0x46
#define DEVICE 1
#define CONFIGURATION 2	 
#define SET_ENDP6 0x1c
#define SET_ENDP7 0x1d
#define AUTO_SETUP 0x4d
#define ISSUE_TKN_X 0x4e
#define SET_USB_SPEED 0x04
#define SET_SD0_INT 0x0b

#define USB_INT_CONNECT 0x15
#define USB_INT_DISCONNECT 0x16
#define USB_INT_SUCCESS 0x14
#define USB_INT_BUF_OVER 0x17

#define CMD_RET_SUCCESS 0x51
#define CMD_RET_ABORT 0x5f

#define USB_INT_BUS_RESET4 0x0f

#define DISCONNECTED 0
#define CONNECTED 1


/*
*
*
*
**/

void reset_all_pin(void){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = XBC_CTL_AO | XBC_CTL_RD | XBC_CTL_WR | XBC_CTL_PCS ;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = CH376_HW_RST | CH376_INT_N;	//hardware reset pin 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = XBC_DATA_D6 | XBC_DATA_D7 | XBC_DATA_D2 | XBC_DATA_D3 | XBC_DATA_D4 | XBC_DATA_D5 ;
	GPIO_Init(XBC_DATA_PORT_2_7, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = XBC_DATA_D6 | XBC_DATA_D7 | XBC_DATA_D2 | XBC_DATA_D3 | XBC_DATA_D4 | XBC_DATA_D5 ;
	GPIO_Init(XBC_DATA_PORT_2_7, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = XBC_DATA_D0 | XBC_DATA_D1;
	GPIO_Init(XBC_DATA_PORT_0_1, &GPIO_InitStructure);

}
void gpio_but_init(void){
	
	GPIO_InitTypeDef GPIO_InitStructure;
		
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	/*
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0); 
	*/
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = P0 | P1 | P7 | P5 | P6;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = P2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_Out_PP; //SW reset pin 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);
		
}

void gpio_ctl_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
		
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	   
  	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	   
  	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = XBC_CTL_AO | XBC_CTL_RD | XBC_CTL_WR | XBC_CTL_PCS ;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	   
	GPIO_InitStructure.GPIO_Pin = CH376_HW_RST;	//hardware reset pin 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = CH376_INT_N;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	external_Interrup_Init(8, GPIOA);
	//external_Interrup_Init(3, GPIOA);
	GPIO_ResetBits(GPIOC,GPIO_Pin_11);
}


void gpio_data_out_init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;  	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = XBC_DATA_D6 | XBC_DATA_D7 | XBC_DATA_D2 | XBC_DATA_D3 | XBC_DATA_D4 | XBC_DATA_D5 ;
	GPIO_Init(XBC_DATA_PORT_2_7, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = XBC_DATA_D0 | XBC_DATA_D1;
	GPIO_Init(XBC_DATA_PORT_0_1, &GPIO_InitStructure);
}

void gpio_data_in_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	  	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = XBC_DATA_D6 | XBC_DATA_D7 | XBC_DATA_D2 | XBC_DATA_D3 | XBC_DATA_D4 | XBC_DATA_D5 ;
	GPIO_Init(XBC_DATA_PORT_2_7, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = XBC_DATA_D0 | XBC_DATA_D1;
	GPIO_Init(XBC_DATA_PORT_0_1, &GPIO_InitStructure);

}

void gpio_data_clear(void)
{
	GPIO_ResetBits(XBC_DATA_PORT_0_1, XBC_DATA_D0);
	GPIO_ResetBits(XBC_DATA_PORT_0_1, XBC_DATA_D1);
	GPIO_ResetBits(XBC_DATA_PORT_2_7, XBC_DATA_D2);
	GPIO_ResetBits(XBC_DATA_PORT_2_7, XBC_DATA_D3);
	GPIO_ResetBits(XBC_DATA_PORT_2_7, XBC_DATA_D4);
	GPIO_ResetBits(XBC_DATA_PORT_2_7, XBC_DATA_D5);
	GPIO_ResetBits(XBC_DATA_PORT_2_7, XBC_DATA_D6);
	GPIO_ResetBits(XBC_DATA_PORT_2_7, XBC_DATA_D7);
}


void ch376_hardware_reset(void){

	GPIO_SetBits(GPIOA, CH376_HW_RST);	//hardware reset //active high	
	_delay_ms(10);
	GPIO_ResetBits(GPIOA, CH376_HW_RST);
	_delay_ms(20);

}

void ch376_software_reset(void){

	ch376_write_cmd(RESET_ALL);
	_delay_ms(5);
}

void ch376_set_parallel_mod(void){

	GPIO_SetBits(GPIOB, XBC_CTL_WR);	//WR -> 1
	GPIO_SetBits(GPIOB, XBC_CTL_RD);	//RD -> 1
	GPIO_SetBits(GPIOB, XBC_CTL_PCS);	//PCS -> 1
	_delay_ms(2);	
	
	//select this chip but do not function
	GPIO_ResetBits(GPIOB, XBC_CTL_PCS);	//PCS -> 0
	GPIO_SetBits(GPIOB, XBC_CTL_RD);    //RD -> 1	
	GPIO_SetBits(GPIOB, XBC_CTL_WR);	//WR -> 1	
	
}


void ch376_hardware_init(void) {
		
	//gpio_but_init();
	gpio_ctl_init();
	gpio_data_out_init();	
	ch376_set_parallel_mod();
	ch376_hardware_reset();
	ch376_software_reset();
	
	

}

/*****			END config. port		     										*******/
/*************************************************************************************/

/*
* 
* paralle port read & write function
*
**/
void data_write_pp(u8 data){

	//(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1))
	gpio_data_out_init();
	GPIO_WriteBit(XBC_DATA_PORT_0_1, XBC_DATA_D0, (BitAction) ((_BV(0) & data) >> 0));
	GPIO_WriteBit(XBC_DATA_PORT_0_1, XBC_DATA_D1, (BitAction) ((_BV(1) & data) >> 1));
	GPIO_WriteBit(XBC_DATA_PORT_2_7, XBC_DATA_D2, (BitAction) ((_BV(2) & data) >> 2));
	GPIO_WriteBit(XBC_DATA_PORT_2_7, XBC_DATA_D3, (BitAction) ((_BV(3) & data) >> 3));
	GPIO_WriteBit(XBC_DATA_PORT_2_7, XBC_DATA_D4, (BitAction) ((_BV(4) & data) >> 4));
	GPIO_WriteBit(XBC_DATA_PORT_2_7, XBC_DATA_D5, (BitAction) ((_BV(5) & data) >> 5));
	GPIO_WriteBit(XBC_DATA_PORT_2_7, XBC_DATA_D6, (BitAction) ((_BV(6) & data) >> 6));
	GPIO_WriteBit(XBC_DATA_PORT_2_7, XBC_DATA_D7, (BitAction) ((_BV(7) & data) >> 7));

}

u8 read_data_pp(void){
u8 data = 0;
	gpio_data_in_init();
	data |=(GPIO_ReadInputDataBit(XBC_DATA_PORT_0_1, XBC_DATA_D0));
	data |=(GPIO_ReadInputDataBit(XBC_DATA_PORT_0_1, XBC_DATA_D1)<<1);
	data |=(GPIO_ReadInputDataBit(XBC_DATA_PORT_2_7, XBC_DATA_D2)<<2);
	data |=(GPIO_ReadInputDataBit(XBC_DATA_PORT_2_7, XBC_DATA_D3)<<3);
	data |=(GPIO_ReadInputDataBit(XBC_DATA_PORT_2_7, XBC_DATA_D4)<<4);
	data |=(GPIO_ReadInputDataBit(XBC_DATA_PORT_2_7, XBC_DATA_D5)<<5);
	data |=(GPIO_ReadInputDataBit(XBC_DATA_PORT_2_7, XBC_DATA_D6)<<6);
	data |=(GPIO_ReadInputDataBit(XBC_DATA_PORT_2_7, XBC_DATA_D7)<<7);	
	return data;
}
/*****									  										*******/
/*************************************************************************************/

/*
* data processing: control ch376 read/write
* 
* 
* 
**/
void ch376_write_cmd (u8 cmd){
	
	data_write_pp(cmd);
	GPIO_SetBits(GPIOB, XBC_CTL_AO);	//A0 -> 1	
	ns_delay();	
	
	GPIO_ResetBits(GPIOB, XBC_CTL_WR);	//WR -> 0
	GPIO_SetBits(GPIOB, XBC_CTL_RD);    //RD -> 1	
	ns_delay();
	ns_delay();
//	ns_delay();
	
	ch376_no_operation();
	
}

void ch376_write_data (u8 data){

	data_write_pp(data);
	
	GPIO_ResetBits(GPIOB, XBC_CTL_AO);	//A0 -> 0
	ns_delay();
	GPIO_ResetBits(GPIOB, XBC_CTL_WR);	//WR -> 0	
	//ns_delay();	
	ns_delay();
	ns_delay();
//	ns_delay();
	
	ch376_no_operation();
}

u8 ch376_read_data (void){
u8 data = 0;

	GPIO_ResetBits(GPIOB, XBC_CTL_AO);	//A0 -> 0
	GPIO_SetBits(GPIOB, XBC_CTL_WR);	//WR -> 1
	ns_delay();
	GPIO_ResetBits(GPIOB, XBC_CTL_RD);	//RD -> 0
	ns_delay();
	ns_delay();
	
	data = read_data_pp();
	ch376_no_operation();	
	return data;
	
}

void ch376_no_operation(void){

	GPIO_SetBits(GPIOB, XBC_CTL_WR);	//WR -> 1
	GPIO_SetBits(GPIOB, XBC_CTL_RD);    //RD -> 1	
	ns_delay();
	
}

void ch376_write_cmd_data(u8 cmd, u8 data){

	ch376_write_cmd(cmd);
	ch376_write_data(data);

}

void ch376_write_cmd_data2(u8 cmd, u8 data0, u8 data1){
	
	ch376_write_cmd(cmd);
	ch376_write_data(data0);
	ch376_write_data(data1);

}

u8 ch376_read_cmd_data(u8 cmd, u8 data){
		
	ch376_write_cmd(cmd);
	
	_delay_ms(5);
	ch376_write_data(data);
	_delay_ms(5);
	
	return ch376_read_data();

}


void ch376_set_sd0_init(void){

	ch376_write_cmd(SET_SD0_INT);
	
	ch376_write_data(0x16);
	_delay_ms(2);
	ch376_write_data(0x90);

}

/*
u8 usb_to_ch376_read(void){
	read data buffer from usb 
	64 bytes data

	ch376_write_cmd(RD_USB_DATA0);
	
	for (i=0;i<64;i++){	
		read_data_buf[i] = ch376_read_data();		
	}
	
	return read_data_buf[0];

}
*/

/*****						END of data proccssing  							*******/
/*************************************************************************************/

u8 ch376_set_addr(u8 addr){
u8	status;
	
	ch376_write_cmd_data(CH376_CMD_SET_ADDRESS,addr);
	status = ch376_get_int_status();
		if (status == CH376_USB_INT_SUCCESS)
			ch376_write_cmd_data(CH376_CMD_SET_USB_ADDR, addr);
	return status;
}

void ch376_set_retry(u8 times)
{
	ch376_write_cmd_data2(CH376_CMD_SET_RETRY, CH376_DATA0_SET_RETRY, times);
}

void ch376_set_usb_speed (u8 speed)
{		
	ch376_write_cmd_data(SET_USB_SPEED,speed);
}


u8 ch376_get_int_status(void){
u8 status = 0;	
	ch376_write_cmd(CH376_CMD_GET_STATUS);
	status = ch376_read_data();			
	return status;	
}


u8 ch376_set_usb_mode(u8 mode){

	ch376_write_cmd_data(SET_USB_MODE,mode);
	_delay_us(20);
	return  ch376_read_data();	
}

u8 ch376_check_exist(u8 data){
/* 
 *check communication between mcu and ch376	 
 */
	return ch376_read_cmd_data(CH376_CMD_CHECK_EXIST, data);

}


u8 ch376_test_connect(void)
{	
	ch376_write_cmd(CH376_CMD_TEST_CONNECT);
	_delay_us(5);
	return ch376_read_data();
}

void write_data_to_usb(u8 len, u8 data[]){
u8 cnt;

	ch376_write_cmd(CH376_CMD_WR_HOST_DATA);
	ch376_write_data(len);
	
	for(cnt = 0; cnt < len; cnt++){	
		ch376_write_data(data[cnt]);	
	}
}

__asm void ns_delay(void)
{
	NOP
}


void EXTI9_5_IRQHandler(void){
//	if (finish_init){
		if (EXTI_GetITStatus(GPIO_Pin_8) != 0){
			if( GPIO_ReadInputDataBit( GPIOA , GPIO_Pin_8 ) == 0 ){
				//test_exti = 0;
//				read_usb_buff();
			//	printf("yeah! %x\r\n",usb_get_status());
			}	
		}
//	}
	EXTI_ClearFlag( EXTI_Line8 );
}

void EXTI3_IRQHandler(void){
	
	if (EXTI_GetITStatus(GPIO_Pin_3) != RESET){
		if( GPIO_ReadInputDataBit( GPIOA , GPIO_Pin_3 ) == 0 ){
			//test_exti = 0;		
		}
	
	}
	EXTI_ClearFlag( EXTI_Line3 );
}
