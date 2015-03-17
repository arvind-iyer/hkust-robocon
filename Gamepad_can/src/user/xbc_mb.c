#include "xbc.h"

u8 xbc_state = 0;
//u8 init_state = 0;
//u8 xbc_vibrate_big_motor = 0;
//u8 xbc_vibrate_small_motor = 0;
u16 xbc_counter = 0;
u8 spi_data=0;
u8 xbc_tft_x_pos=0;
u8 xbc_tft_y=0;
u8 xbc_tft_section=0;
u8 transmit_mode=0; 
u8 xbc_done=0;
u8 tft_done=0;
u8 is_big_controller=0;
u8 is_xbc_controller=0;

//public data
u8 xbc_mode = 0 ;
u8 volatile old_xbc_mode = 0;
u8 volatile xbc_data[XBC_DATA_LENGTH]={0};
u8 volatile xbc_down[3];
u8 volatile xbc_up[3];
u8 xbc_error_count = 0;

void xbc_init(void){
	SPI_InitTypeDef SPI_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2 , ENABLE );
	xbc_gpio_init();
	
	SPI_StructInit(&SPI_InitStructure );
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_I2S_DeInit(XBC_SPI); 
	SPI_Cmd(XBC_SPI, DISABLE);  
	SPI_Init(XBC_SPI, &SPI_InitStructure);
	SPI_SSOutputCmd( XBC_SPI , ENABLE );
	SPI_CalculateCRC(XBC_SPI, DISABLE); 
	SPI_StructInit( &SPI_InitStructure );
	SPI_Cmd(XBC_SPI, ENABLE);
	
	xbc_timer_init();
	xbc_chip_select();
}

void xbc_gpio_init(void){
	GPIO_InitTypeDef IO_2;
	GPIO_InitTypeDef IO_1;
	GPIO_InitTypeDef IO_3;
	/* Configure SPI1 pins: SCK , MISO and MOSI */
	IO_1.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	IO_2.GPIO_Pin = GPIO_Pin_14;
	IO_1.GPIO_Speed = GPIO_Speed_50MHz;
	IO_2.GPIO_Speed = GPIO_Speed_50MHz;

	/* Configure SCK and MOSI pins as Alternate Function Push Pull,
		 MISO pin as Input Floating		*/
	IO_1.GPIO_Mode = GPIO_Mode_AF_PP;
	IO_2.GPIO_Mode = GPIO_Mode_IPU;			

	GPIO_Init(XBC_SPI_PORT, &IO_1);
	GPIO_Init(XBC_SPI_PORT, &IO_2);
	
	IO_1.GPIO_Speed = GPIO_Speed_50MHz;
	IO_1.GPIO_Pin = XBC_NSS ;
	IO_1.GPIO_Mode = GPIO_Mode_Out_PP;
	
	IO_3.GPIO_Mode = GPIO_Mode_IPD;
	IO_3.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOD, &IO_3);
	
	GPIO_Init(XBC_SPI_PORT, &IO_1);
	GPIO_SetBits( XBC_SPI_PORT , XBC_NSS );
}

u8 get_xbc_mode(){
	return xbc_mode;
}

void xbc_check_flag (void) 
{
	u8 i ;	
	if (xbc_mode==0 && old_xbc_mode!=0)
	{
		/*xbc_data[XBC_BUT1]=0xFF;
		xbc_data[XBC_BUT2]=0xFF;
		xbc_data[XBC_BUT3]=0xFF;*/
	} else if (xbc_mode!=0 && old_xbc_mode==0)
	{
		/*old_xbc_data[XBC_BUT1]=0xFF;
		old_xbc_data[XBC_BUT2]=0xFF;
		old_xbc_data[XBC_BUT3]=0xFF;*/
	}
	
	if (xbc_mode < 6 && old_xbc_mode >= 6)
	{
		/*xbc_data[XBC_JOYRIGHTX]=0x80;
		xbc_data[XBC_JOYRIGHTY]=0x80;
		xbc_data[XBC_JOYLEFTX]=0x80;
		xbc_data[XBC_JOYLEFTY]=0x80;*/
	}
				
	// ^^^ up : when pressed
	xbc_up[XBC_BUT1] = xbc_data[XBC_BUT1] & (xbc_data[XBC_BUT1] ^ old_xbc_data[XBC_BUT1]);
	xbc_up[XBC_BUT2] = xbc_data[XBC_BUT2] & (xbc_data[XBC_BUT2] ^ old_xbc_data[XBC_BUT2]);
	xbc_up[2] = xbc_data[2] & (xbc_data[2] ^ old_xbc_data[2]);
	
	// ^^^ down : when released
	xbc_down[XBC_BUT1] = old_xbc_data[XBC_BUT1] & (xbc_data[XBC_BUT1] ^ old_xbc_data[XBC_BUT1]);
	xbc_down[XBC_BUT2] = old_xbc_data[XBC_BUT2] & (xbc_data[XBC_BUT2] ^ old_xbc_data[XBC_BUT2]);
	xbc_down[2] = old_xbc_data[2] & (xbc_data[2] ^ old_xbc_data[2]);
	
	for(i = 0; i < XBC_DATA_LENGTH; i++)
		old_xbc_data[i] = xbc_data[i];
	old_xbc_mode = xbc_mode;
}							

void xbc_tft_transmit(void){
	if( xbc_state != 0 )
		spi_data = SPI_I2S_ReceiveData(XBC_SPI);
	if(is_big_controller)
		transmit_mode=3;
	else
		transmit_mode=4;
	switch( xbc_state  ){
		case 0:
			xbc_chip_select();
			SPI_I2S_SendData(XBC_SPI , XBC_HELLO);
			xbc_state ++;
			break;
		case 1:
			SPI_I2S_SendData(XBC_SPI , COMMAND_TFT);
			xbc_state ++;
			break;		
		case 2:				
			if (spi_data == 0xFF || spi_data == 0x00) 
			{
				xbc_state = 0;
				if(  xbc_error_count ++ > 50 ){
					xbc_chip_deselect();
					transmit_mode=0;
					xbc_mode = 0;
					xbc_error_count = 0;
				}
			} else{
				xbc_error_count = 0;	
				SPI_I2S_SendData(XBC_SPI, XBC_COMFIRMED);
				xbc_state ++;
			}
			break;
			
		default: // state = 3 ++
		
	/*---Variable Definition---
			17:1 line number + 16 tft data
			xbc_tft_section:upper 5 lines or lower 5 lines
			xbc_state:-state of the transmission, the position of the transmitted byte
					  -total 88 bytes:3+16*5+5; 3:connection construction bytes; 16*5:tft data for 5 lines; 5:bytes for line numbers
			xbc_tft_y:line number
			xbc_tft_x_pos:x position of a byte on a line		
			***TRY TO THINK OF THE MATHS***
					---Variable Definition---*/
		
			tft_done++;
			xbc_tft_x_pos=(xbc_state-3)%17;
			if(xbc_tft_x_pos ==0 ){				
				xbc_tft_y=xbc_tft_section%2?(xbc_state-3)/17:(xbc_state-3)/17+5;
				SPI_I2S_SendData( XBC_SPI , XBC_TFT_LINE | xbc_tft_y ); 
			}				
			else{
				SPI_I2S_SendData( XBC_SPI , text[xbc_tft_y][xbc_tft_x_pos-1] );
			}
			xbc_state++;
			if(xbc_state>=88){				
				xbc_state = 0;
				xbc_tft_section++;
			//	xbc_chip_deselect();	disabled chip deselect otherwise the last byte for each chunk of data will be lost
				transmit_mode=4;
			}
			break;
	}
}

void get_xbc_data(void){
	static u8 spi_data_length = 0;
	if( xbc_state != 0 )
		spi_data = SPI_I2S_ReceiveData(XBC_SPI);
	transmit_mode=1;
	switch( xbc_state ){
		case 0:
			xbc_chip_select();
			SPI_I2S_SendData(XBC_SPI , XBC_HELLO);
			xbc_state ++;
			break;
		case 1:
			SPI_I2S_SendData(XBC_SPI , COMMAND_XBC);
			xbc_state ++;
			break;
		case 2:
			if (spi_data == 0xFF || spi_data == 0x00) 
			{
				xbc_state = 0;
				xbc_chip_deselect();
				if(  xbc_error_count ++ > 50 ){
					transmit_mode=0;
					xbc_mode = 0;
					xbc_error_count = 0;
					xbc_check_flag();
				}
			} else{
				xbc_error_count = 0;	
				SPI_I2S_SendData(XBC_SPI, XBC_COMFIRMED);
				spi_data_length = (spi_data & 0x0F) << 1;
				if(spi_data == BIG_CONTROLLER){
					spi_data_length++;
					is_big_controller=1; //use to tell mcu sell tft to lcd
				}
				else if (spi_data == XBOX_CONTROLLER){
					spi_data_length+=4; //=12
					is_xbc_controller=1;
				}
				xbc_mode = spi_data_length;
				xbc_state ++;
			}
			break;
		default: // state = 3 ++
			//xbc_done++;
			if( spi_data_length -- ){
				SPI_I2S_SendData( XBC_SPI , 0x00 );
				if( xbc_state >= 4)
					xbc_data[xbc_state - 4] = spi_data;
				xbc_state ++;
			}
			else{			
				if( xbc_state >= 4 )
					xbc_data[xbc_state - 4] = spi_data;
				xbc_state = 0;
	//			xbc_chip_deselect();
				transmit_mode=2;
			}
	}	
	}
}

void TIM7_IRQHandler(void){
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	switch(transmit_mode){
		case 0:
			transmit_mode++;
			break;
		case 1:
			get_xbc_data();	
			break;
		case 2:
			if( xbc_counter ++ > 40 ){
				xbc_counter = 0;
				//transmit_mode=3;
				transmit_mode=0;
			}			
			break;
		case 3:
			xbc_tft_transmit();		
			break;
		case 4:
			if( xbc_counter ++ > 40 ){
				xbc_counter = 0;
				transmit_mode=0;
			}
			break;
		default:
			break;		
	}
}

void xbc_timer_init(void){
	
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);
	TIM7->ARR = 8 * 2 + 14;
	TIM7->XBC = 256 - 1 ;
	TIM7->EGR = 1;
	TIM7->SR = 0;
	TIM7->DIER = 1;
	TIM7->CR1 = 1;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn; 
	NVIC_Init(&NVIC_InitStructure);
}

void xbc_chip_select(void){
	GPIO_ResetBits(XBC_SPI_PORT, XBC_NSS);
	_delay_us(3);
}

void xbc_chip_deselect(void){
	GPIO_SetBits(XBC_SPI_PORT, XBC_NSS);
}

/* for the hiden mysterious analog function for xbc
void analog_mode_init(void){
	u8 into_config_mode[5] = { XBC_HELLO , BIG_CONTROLLER , 0x00 , XBC_HELLO , 0x00 };
	u8 turn_on_analog_mode[9] = { XBC_HELLO, 0x44, 0x00, XBC_HELLO, 0x03, 0x00, 0x00, 0x00, 0x00};
	u8 motor_command_mapping[9] = { XBC_HELLO, 0x4D, 0x00, 0x00, XBC_HELLO, 0xFF, 0xFF, 0xFF, 0xFF };
	u8 return_pressure_val[9] = { XBC_HELLO, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00};
	u8 exit_config_mode[9] = { XBC_HELLO, BIG_CONTROLLER, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };
	
	u8 length = 9;
	u8 * array_pointer=0;
	switch( init_state ){
		case 0: 
			array_pointer = into_config_mode; 
			length = 5;
		break;
		case 1: array_pointer = turn_on_analog_mode;
		break;
		case 2: array_pointer = motor_command_mapping;
		break;
		case 3: array_pointer = return_pressure_val;
		break;
		case 4: array_pointer = exit_config_mode;
		break;
		
	}
	
	if( xbc_state == 0){
			GPIO_ResetBits( XBC_SPI_PORT , XBC_NSS );
		_delay_us(3);
	}
	if( xbc_state != length)
		SPI_I2S_SendData( XBC_SPI , array_pointer[xbc_state++] );
	else{
		xbc_chip_deselect();
		xbc_state = 0;
		init_state ++;
	}
	
	switch( init_state ){
		case 0:
			if( state == 0){
				xbc_chip_select();
				_delay_us(3);
			}
			if( state != 5)
				SPI_I2S_SendData( XBC_SPI , into_config_mode[state++] );
			else{
				xbc_chip_deselect();
				state = 0;
				init_state ++;
			}
		break;
		case 1:
			if( state == 0){
				xbc_chip_select();
				_delay_us(3);
			}
			if( state != 9)
				SPI_I2S_SendData( XBC_SPI , turn_on_analog_mode[state++] );
			else{
				xbc_chip_deselect();
				state = 0;
				init_state ++;
			}
		break;
		case 2:
			if( state == 0){
				xbc_chip_select();
				_delay_us(3);
			}
			if( state != 9)
				SPI_I2S_SendData( XBC_SPI , motor_command_mapping[state++] );
			else{
				xbc_chip_deselect();
				state = 0;
				init_state ++;
			}
		break;
		case 3:
			if( state == 0){
				xbc_chip_select();
				_delay_us(3);
			}
			if( state != 9)
				SPI_I2S_SendData( XBC_SPI , return_pressure_val[state++] );
			else{
				xbc_chip_deselect();
				state = 0;
				init_state ++;
			}
		break;
		case 4:
			if( state == 0){
				xbc_chip_select();
				_delay_us(3);
			}
			if( state != 9)
				SPI_I2S_SendData( XBC_SPI , exit_config_mode[state++] );
			else{
				xbc_chip_deselect();
				state = 0;
				init_state ++;
			}
		break;
	}
}
*/
