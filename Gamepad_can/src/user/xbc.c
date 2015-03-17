#include "xbc.h"

u8  xbc_state = 0;
u32 xbc_counter = 0;
u32 xbc_received = 0;
u8 data_busy = 0;
u8 spi_failure = 0;
s8 lcd_current_line = -1;
s8 lcd_cursor = -1;
u8 lcd_command = 0;
u8 lcd_command_data = 0 ;
u8 lcd_temp_data[17] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, '\0'};
u8 command = 0;
u8 data;
u8 buf_cnt = 0;
u8 xbc_data[20];
u8 tx_xbc_data[20];
u8 is_failure_data = 0; //1 : failure data, 0 : no
u8 read_data_err[20] = {0};
u16 continues_error = 0; //ch376 will keep reading invaild data until xbc is trigger, this flag try to fix it.

u8 tft_receive_flag = 0;

void xbx_check_flag(void){
	
	is_failure_data = 0;
	if (xbc_data[1] != 0 || xbc_data[0] != 0x14){
	/**according to observe, second data should always keep zero
	//otherwise error data occur**/
	//	xbx_check_flag();
	is_failure_data = 1;				
	}
	else if (xbc_data[13] == 0 && xbc_data[14] == 0 ){ // when both there two data are 0, error reading from digital button
		is_failure_data = 1;				
	}
	else if (xbc_data[15] != 0 || xbc_data[16] != 0){//when the 15th data read more than 0, error reading occur
		is_failure_data = 1;				
	}
}

void usb_read_data_buf(void){
/*
	16 bytes
	-there are 12 bytes data + first byte (data_length) + second data (unknow)
	-change value of second xbc data to 0xa5 as reference for debug/detect	
*/		
u8 i =0;
	is_failure_data = 1;
	//is_xbc_board_processing = 1;
	if (check_usb_data_buff() == USB_STATE_CONFIGURED){		
		//read_usb_buff();
		for (buf_cnt=0;buf_cnt<20;buf_cnt++){				
		//	xbc_data[buf_cnt] = ch376_read_data();				
		}
		if (xbc_data[0] == 0x14 || xbc_data[2] == 0x14){
		//	xbc_data[1] = 0xa5;
			is_failure_data = 0;
		}
	//	is_xbc_board_processing = 0;
	}
	else{		
		for(i=0;i<20;i++){
		//	xbc_data[i] = 0;
		}
		//xbc_data[1] = 0xa5;
		is_failure_data = 1;
	
	}
	//printf("G\r\n");	
}

void SPI2_IRQHandler(void)
{	
	if (SPI_I2S_GetITStatus(XBC_SPI, SPI_I2S_FLAG_RXNE) == RESET){
	SPI_I2S_ClearFlag(XBC_SPI , SPI_I2S_FLAG_RXNE);
	data = SPI_I2S_ReceiveData(XBC_SPI);
	 
		if (data == XBC_HELLO){ //&& command != 0){
			SPI_I2S_SendData(XBC_SPI , XBC_CONTROLLER);
			is_spi_interrupting = 1;
			xbc_state = 0;
	
		}
		else if(xbc_state == 0 && (data == COMMAND_XBC || data== COMMAND_TFT || data== COMMAND_TFT_CL)){
			SPI_I2S_SendData(XBC_SPI , XBC_HANDSHAKE );
			command = data;
			xbc_state ++;
			data_busy = 1;
			//buf_cnt = 3;
			
		//	printf("spi OK");
		}
		else if (xbc_state >= 1 ){
		
			if(command == COMMAND_XBC && data == XBC_COMFIRMED){
				
				SPI_I2S_SendData(XBC_SPI, tx_xbc_data[xbc_state-1]);
				
				if(xbc_state++ == 15 ){ // only first 15bytes data useful				
					//buf_cnt = 0;
					data_busy = 0;
				}
			}
			else if(command == COMMAND_TFT){ //&& !enabled_system_tft){ 
				SPI_I2S_SendData( XBC_SPI , XBC_HANDSHAKE );
				if( data >= XBC_TFT_LINE){ // check if this data is line number
					lcd_command_data = data & 0x0F;
					lcd_current_line = lcd_command_data;
					lcd_cursor = 0;
				//	printf("L:%x\r\n",lcd_current_line);
				}
				else{ // read data from mainborad
					lcd_temp_data[lcd_cursor++] = data;
				//	printf("%x  *",data);
				//	printf("W:%x ",lcd_temp_data[lcd_cursor]);
					if(lcd_cursor>=16){
				//		printf("\r\n");
						lcd_cursor = 0;
						tft_prints(0 , lcd_current_line , lcd_temp_data );						
						if (lcd_current_line == 4 || lcd_current_line == 9){
							data_busy=0;
							tft_receive_flag = 1;
						}
					}					
				}
			}
			else if(command == COMMAND_TFT_CL){ //&& !enabled_system_tft){
				SPI_I2S_SendData( XBC_SPI , XBC_HANDSHAKE );
				if( data >= XBC_TFT_LINE){ // check if this data is line number
					lcd_command_data = data & 0x0F;
					lcd_current_line = lcd_command_data;
					lcd_cursor = 0;
				//	printf("L:%x\r\n",lcd_current_line);
				}
				else{
					text_bg_color_prev[lcd_cursor++][lcd_current_line] = data;
				//	printf("%x  |",data);
					
				//	lcd_temp_data[lcd_cursor++] = data;
				//	printf("W:%x ",lcd_temp_data[lcd_cursor]);
			
					if(lcd_cursor>=16){
				//		printf("\r\n");
						lcd_cursor = 0;						
						if (lcd_current_line == 4 || lcd_current_line == 9){
							data_busy=0;
							command = 0;
						}
					}
					
				}
			}
			else{
				SPI_I2S_SendData(XBC_SPI , 0);
			}
			
		}
		else{		
			xbc_state = 0;
			data_busy = 0;
			spi_failure = 1;
			printf("spi not OK");
		
		}	
		}	
		
}

u8 reset_spi_transmit_flag(void){

	//xbc_state = 0;
	data_busy = 0;
	spi_failure = 1;
	command = 0;		

}

s8 get_line_number(void){

	return lcd_current_line;

}

void reset_line_number(void){

	lcd_current_line = 0;
	
}

void spi_init (void){

	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	// init GPIO	
	GPIO_InitStructure.GPIO_Pin = XBC_MISO;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(XBC_GPIO, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = XBC_SCK | XBC_MOSI ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(XBC_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = XBC_NSS ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(XBC_GPIO, &GPIO_InitStructure);
	
	GPIO_SetBits(XBC_GPIO, XBC_NSS);
	
	// init SPI
	SPI_StructInit(&SPI_InitStructure );
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
 	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;	
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_Init(XBC_SPI, &SPI_InitStructure);

//	SPI_SSOutputCmd( XBC_SPI , ENABLE );
	SPI_CalculateCRC(XBC_SPI, DISABLE); 
	SPI_Cmd(XBC_SPI, ENABLE);	
	
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
  
	// init interrupt
	SPI_I2S_ITConfig(XBC_SPI, SPI_I2S_IT_RXNE, ENABLE);
	SPI_I2S_ClearFlag( XBC_SPI , SPI_I2S_FLAG_RXNE);		
	_delay_ms(10);
}


void xbc_chip_select(void){
	GPIO_ResetBits(XBC_GPIO, XBC_NSS);
	_delay_us(3);
}

void xbc_chip_deselect(void){
	GPIO_SetBits(XBC_GPIO, XBC_NSS);
}