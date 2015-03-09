#include "add_lib_v0.h"

/***************************************************************************************************************************/


/**************************************************************************************************************************/

u8 GET_DESCR_CMD[8] = { 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00 };
u8 SET_DESCR_CMD[8] = { 0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x04, 0x00 };
u8 SET_USB_ADDR_CMD[8] = { 0x00, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 SET_USB_CONFIG_CMD[8] = { 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

u8 read_data_buf[65] = {0}; 
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
#define _12M 0x00
#define _1_5M_FULL 0x01
#define _1_5M_LOW 0x02
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




void gpio_g1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
		
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	//MCU G1 -> A0 
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = CH376_HW_RES;	//hardware reset pin 
	GPIO_Init(GPIOE, &GPIO_InitStructure);
		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = CH376_INT_N;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
}

void gpio_b_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	//MCU B7 -> WR
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	//MCU B8 -> RD 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	//MCU B9 -> PCS	 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void gpio_f_out_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	//MCU F1 -> D0 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 	//MCU F2 -> D1
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	//MCU F3 -> D2 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	//MCU F4 -> D3 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	//MCU F5 -> D4
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	//MCU F13 -> D5 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;	//MCU F14 -> D6 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	//MCU F15 -> D7 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}

void gpio_f_in_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	//MCU F1 -> D0 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 	//MCU F2 -> D1
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	//MCU F3 -> D2 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	//MCU F4 -> D3 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	//MCU F5 -> D4
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	//MCU F13 -> D5 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;	//MCU F14 -> D6 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	//MCU F15 -> D7 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}

void gpio_f_clear(void)
{
	GPIO_ResetBits(GPIOF, GPIO_Pin_1);
	GPIO_ResetBits(GPIOF, GPIO_Pin_2);
	GPIO_ResetBits(GPIOF, GPIO_Pin_3);
	GPIO_ResetBits(GPIOF, GPIO_Pin_4);
	GPIO_ResetBits(GPIOF, GPIO_Pin_5);
	GPIO_ResetBits(GPIOF, GPIO_Pin_13);
	GPIO_ResetBits(GPIOF, GPIO_Pin_14);
	GPIO_ResetBits(GPIOF, GPIO_Pin_15);
}





void port_init(void){

	gpio_g1_init();	
	gpio_b_init();
	gpio_f_out_init();
	
	ch376_set_parallel_mod();
	hardware_reset();

}


void hardware_reset(void){

	GPIO_SetBits(GPIOE, GPIO_Pin_0);	//hardware reset //active high	
	_delay_ms(10);
	GPIO_ResetBits(GPIOE, GPIO_Pin_0);
	_delay_ms(40);

}


void ch376_set_parallel_mod(void){

	GPIO_SetBits(GPIOB, GPIO_Pin_7);	//WR -> 1
	GPIO_SetBits(GPIOB, GPIO_Pin_8);	//RD -> 1
	GPIO_SetBits(GPIOB, GPIO_Pin_9);	//PCS -> 1
	_delay_ms(2);	
	
	//select this chip but do not function
	GPIO_ResetBits(GPIOB, GPIO_Pin_9);	//PCS -> 0
	GPIO_SetBits(GPIOB, GPIO_Pin_8);    //RD -> 1	
	GPIO_SetBits(GPIOB, GPIO_Pin_7);	//WR -> 1
	
	
}

void ch376_no_operation(void){

	GPIO_SetBits(GPIOB, GPIO_Pin_7);	//WR -> 1
	GPIO_SetBits(GPIOB, GPIO_Pin_8);    //RD -> 1	
	ns_delay();
	
}

void ch376_write_cmd (u8 cmd){
	
	ch376_read_data();
	ch376_read_data();
	ch376_read_data();
	
	xbc_data_write(cmd);
	GPIO_SetBits(GPIOG, GPIO_Pin_1);	//A0 -> 1	
	ns_delay();	
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_7);	//WR -> 0
	GPIO_SetBits(GPIOB, GPIO_Pin_8);    //RD -> 1	
	ns_delay();
	ns_delay();
	ns_delay();
	
	ch376_no_operation();
	
}

void ch376_write_cmd_data (u8 data){

	ch376_read_data();
	ch376_read_data();
	ch376_read_data();

	xbc_data_write(data);
	
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);	//A0 -> 0
	ns_delay();
	GPIO_ResetBits(GPIOB, GPIO_Pin_7);	//WR -> 0	
	//ns_delay();	
	ns_delay();
	ns_delay();
	ns_delay();
	
	ch376_no_operation();
}


u8 ch376_read_data (void){
u8 data = 0;

	GPIO_ResetBits(GPIOG, GPIO_Pin_1);	//A0 -> 0
	GPIO_SetBits(GPIOB, GPIO_Pin_7);	//WR -> 1
	ns_delay();
	GPIO_ResetBits(GPIOB, GPIO_Pin_8);	//RD -> 0
	ns_delay();
	ns_delay();
	
	data = xbc_data_read();
	ch376_no_operation();	
	return data;
	
}

void ch376_write_cmd1(u8 cmd, u8 data){

	ch376_write_cmd(cmd);
	ch376_write_cmd_data(data);

}

void ch376_write_cmd2(u8 cmd, u8 data1, u8 data2){
	
	ch376_write_cmd(cmd);
	ch376_write_cmd_data(data1);
	ch376_write_cmd_data(data2);

}

void software_reset(void){

	ch376_write_cmd(RESET_ALL);
	_delay_ms(40);
}

void usb_setup_speed (u8 speed){

	ch376_write_cmd(SET_USB_SPEED);
	
	ch376_write_cmd_data(speed);

}

usb_write_cmd(u8 data_length){

	ch376_write_cmd(WR_HOST_DATA);
	
	ch376_write_cmd_data(data_length);
	
	for(i = 0; i < data_length; i++){  		
		ch376_write_cmd_data(GET_DESCR_CMD[i]);
		simple_delay10_us();
	}
	
}

u8 usb_read(void){

	ch376_write_cmd(RD_USB_DATA0);
	
	for (i=0;i<65;i++){	
		read_data_buf[i] = ch376_read_data();		
	}
	
	return read_data_buf[0];

}


u8 get_int_status(void){

u8 state = 0;

	ch376_write_cmd(GET_STATUS);
	
	/*for (i=0;i<64;i++){
	
		read_data_buf[i] = ch376_read_data();
		
	}*/
	state = ch376_read_data();	
		
	return state;
	
}


u8 usb_set_mode(u8 mode){

	u8 code = 0;
   	
	ch376_write_cmd(SET_USB_MODE);
	
	ch376_write_cmd_data(mode);
	
	code = ch376_read_data();	

	return code;
	
}

void usb_connect_conf(void){
u8 k = 255;
u8 state = 0;
u8 j = 5;
u16 int_cnt = 0; //test interrupt#

while(1){
	
	button_update();	
	if (!GPIO_ReadInputDataBit(GPIOE,CH376_INT_N))
		int_cnt++;
	
	switch(state){
		
	case 0:	
		tft_prints(0,0,"usb_mode 0x05..");
		tft_update();
		state++;		
	break;
	
	case 1:	
		while(usb_set_mode(0x05) != CMD_RET_SUCCESS);
		
		tft_prints(0,1,"...OK!!");
		tft_prints(0,2,"USB connect..");
				
		state++;			
	break;
	
	
	case 2:		
		if(get_ticks() % 5 == 0 ) {			
				
			if (get_int_status() == USB_INT_CONNECT){
			
				while(usb_set_mode(0x07) != CMD_RET_SUCCESS);				
				
				
					tft_prints(0,2,"USB connect.00");
					
				while(usb_set_mode(0x06) != CMD_RET_SUCCESS);  	 
			   						
					tft_prints(0,2,"USB connect.10");
				while(k--) {
					usb_setup_speed(_1_5M_LOW);
					_delay_us(100);
				if (get_int_status() != USB_INT_SUCCESS)
				{
					tft_prints(0,2,"USB connect.11");
					break;
				}
				}
							
				k=255;
			
				while(k--){
				ch376_write_cmd(0x45); //control trans. set_ADDRASS		
				ch376_write_cmd_data(0x01);
					if (get_int_status() != USB_INT_SUCCESS)
					{
						tft_prints(0,2,"USB connect.12");
						break;
					}
				}
				
				k=255;
				ch376_write_cmd(0x13); //set_USB_ADDR
		
				ch376_write_cmd_data(0x01);
				
				
				while(k--){
				ch376_write_cmd(0x49); //SET_CONFIG
		
				ch376_write_cmd_data(0x01);		
				
				if (get_int_status() != USB_INT_SUCCESS)
					{
						tft_prints(0,2,"USB connect.22");
						break;
					}
				}
		    /*	
				ch376_write_cmd(0x4D);
				if(get_int_status() == USB_INT_SUCCESS)
				tft_prints(0,2,"USB connect..11");
			*/	
				tft_prints(0,3,"connected    ");
				state=10;
			}
			else{ //			
				while(usb_set_mode(0x05) != CMD_RET_SUCCESS);  				
				get_int_status();
				tft_prints(0,2,"USB connect.00");
				tft_prints(0,3,"disconnect");
				tft_prints(0,4,"               ");
			
			}		
		}
	
	break;
	
	
	case 10:		
		if(button_down & WAKEUP){
		
		ch376_write_cmd2(ISSUE_TKN_X, 0x80, 0x19);
		_delay_ms(100);
		printf("\n\r %xH \n\r",get_int_status());
		
		usb_read();			
			for(i=0;i<65;i++)
				printf("%xH",read_data_buf[i]);
				printf("\n\r");
		
		
			/*
			ch376_write_cmd1(GET_DESCR,0x00);
			_delay_ms(100);
			printf("\n\r %xH \n\r",get_int_status());	
			usb_read();	
			printf("\n\r get 0x00\n");	
			for(i=0;i<65;i++)
				printf("%xH",read_data_buf[i]);
				printf("\n\r");
			
			
			ch376_write_cmd1(GET_DESCR,0x01);
			_delay_ms(100);	
			//if (get_int_status() == USB_INT_SUCCESS)
			printf("\n\r %xH \n\r",get_int_status());	
			usb_read();
			printf("\n\rget 0x01\n");	
			for(i=0;i<65;i++)
				printf("%xH",read_data_buf[i]);
				printf("\n\r");
				
			ch376_write_cmd1(GET_DESCR,0x02);
			_delay_ms(100);
			printf("\n\r %xH \n\r",get_int_status());	
			usb_read();	
			printf("\n\r get 0x02\n");	
			for(i=0;i<65;i++)
				printf("%xH",read_data_buf[i]);
				printf("\n\r");
				
		*/
		
		}
	
	break;
	
	
/***************************************************************************************************/	
	case 3:
		
		usb_write_cmd(8);
		
		ch376_write_cmd(ISSUE_TKN_X);		
		
		ch376_write_cmd_data(0x00);
		
		ch376_write_cmd_data(0x0d);
		
		_delay_us(100);
		
		tft_prints(0,4,"transmission %xH",get_int_status());
		
			
		state++;
	break;
	
	
	case 4: //"transmission set
		
		ch376_write_cmd(ISSUE_TKN_X);		
		
		ch376_write_cmd_data(0x80);
		
		ch376_write_cmd_data(0x09);
		
		_delay_us(100);
		
		tft_prints(0,5,"trans.set %xH",get_int_status());
		
		usb_read();
		
		state = 10;		
	break;
	
	
	case 5: //"transmission status
	
		ch376_write_cmd(WR_HOST_DATA);
	
		ch376_write_cmd_data(0);
		
		ch376_write_cmd(ISSUE_TKN_X);		
		
		ch376_write_cmd_data(0x40);
		
		ch376_write_cmd_data(0x01);
		
		_delay_us(100);
		
		tft_prints(0,5,"trans.sat %xH",get_int_status());
		
		state++;
	break;
	
	
	case 6:
	
		if (get_seconds()%50 == 0){
			tft_prints(0,2,"Test receive");
		 	state++;
		}
		
	break;
	
	case 7:
		/*
		ch376_write_cmd(ISSUE_TKN_X);		
		
		ch376_write_cmd_data(0x80);
		
		ch376_write_cmd_data(0x09);
		
		_delay_us(100);
		
		if (get_int_status() != USB_INT_SUCCESS){
			tft_prints(0,2,"Test receive F");
			break;
		}
		else
			tft_prints(0,2,"Test receive T");
			
		*/
		if (get_seconds()%3 == 0){
			usb_read();
		}
		
		
	break;
	
	}
	
	
	tft_prints(0,6,"statu= %xH ",get_int_status());
	
	if (get_int_status() == USB_INT_DISCONNECT) //if disconnect
		state = 2;
	
	ch376_write_cmd(0x16); //TEST_CONNECT	
	tft_prints(0,7,"usb_s=%xH int%d", ch376_read_data(), int_cnt);
	tft_prints(0, 8, "%xH%xH%xH%xH%xH%H",read_data_buf[0],read_data_buf[1],read_data_buf[2],read_data_buf[3],read_data_buf[4],read_data_buf[5]);
	tft_prints(0, 9, "%xH%xH%xH%xH%xH%H",read_data_buf[6],read_data_buf[7],read_data_buf[8],read_data_buf[9],read_data_buf[10],read_data_buf[11]);
	tft_prints(15, 9, "%d", get_seconds() % 10);
	
	
	tft_update();
	
}
}

void set_sd0_int(void){

	ch376_write_cmd(SET_SD0_INT);
	
	ch376_write_cmd_data(0x16);
	_delay_ms(2);
	ch376_write_cmd_data(0x90);
}

void test_menu(void)
{	
	
	port_init();
	software_reset();
	set_sd0_int();
		
	ch376_write_cmd(0x06);
	
	ch376_write_cmd_data(0x0f);
	//_delay_ms(200);
	Data = ch376_read_data();	
	
	usb_connect_conf();
	
	while(1)
	{
		button_update();
		tft_update();
		tft_prints(0,1,"Parallel Interf");
		tft_prints(0,2,"Data Receive");
		tft_prints(0,3,"Data = %xH", Data);
		
		tft_prints(15,9,"%d",get_seconds()%10);
	}
}


void xbc_data_write(u8 data){

	//(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1))
	gpio_f_out_init();
	GPIO_WriteBit(XBC_DATA_PORT, XBC_DATA_D0, (_BV(0) & data) >> 0);
	GPIO_WriteBit(XBC_DATA_PORT, XBC_DATA_D1, (_BV(1) & data) >> 1);
	GPIO_WriteBit(XBC_DATA_PORT, XBC_DATA_D2, (_BV(2) & data) >> 2);
	GPIO_WriteBit(XBC_DATA_PORT, XBC_DATA_D3, (_BV(3) & data) >> 3);
	GPIO_WriteBit(XBC_DATA_PORT, XBC_DATA_D4, (_BV(4) & data) >> 4);
	GPIO_WriteBit(XBC_DATA_PORT, XBC_DATA_D5, (_BV(5) & data) >> 5);
	GPIO_WriteBit(XBC_DATA_PORT, XBC_DATA_D6, (_BV(6) & data) >> 6);
	GPIO_WriteBit(XBC_DATA_PORT, XBC_DATA_D7, (_BV(7) & data) >> 7);

}

u8 xbc_data_read(void){
u8 data = 0;

	gpio_f_in_init();
	data |=(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1));
	data |=(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_2)<<1);
	data |=(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_3)<<2);
	data |=(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_4)<<3);
	data |=(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_5)<<4);
	data |=(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_13)<<5);
	data |=(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_14)<<6);
	data |=(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_15)<<7);	
	return data;
}


__asm void ns_delay(void)
{
	NOP
}