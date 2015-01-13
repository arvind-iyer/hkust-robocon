#include "xbc_mb.h"

#define xbc_rx_mode 0
#define tft_tx_mode 1

u8 lcd_extend_enable = 2;  //2 enable, 3 disable

u8 running = 0;
u8 xbc_state = 0;
u16 xbc_counter = 0;
u8 spi_data=0;
u8 xbc_tft_x_pos=0;
u8 xbc_tft_y=0;
u8 xbc_tft_section=0;
u8 transmit_state=0;
u8 transmit_mode = 0;
u8 xbc_done=0;
u8 tft_done=0;
u8 is_xbc_controller=0;
u8 finish_read_flag = 0;
u8 xbc_error_count = 0;
u8 is_big_controller=0;
u32 old_xbc_digital = 0;
u16 disconnect_cnt = 0;
u8 error_xbc_data_cnt = 0; // count error number

//public data
u32 volatile xbc_press = 0; //edge trigger of digital
u32 volatile xbc_release = 0; //edge trigger of digital
u8 volatile xbc_mode = 1; //assume connect = 1, disconnect = 0
u8 volatile xbc_channel = 1; //channel 1 -4
u8 volatile old_xbc_mode = 0;
u32 volatile xbc_digital=0; //conbine two 8bits variable data into one 16bits
s16 volatile xbc_joy[6]={0}; // combine two joy (eight 8bits, two into one) and two special analog (two 8bit)
u8 volatile xbc_analog = 1;


u8 volatile xbc_data[XBC_DATA_LENGTH]={0};  //store data from xbc
//u8 volatile old_xbc_data[XBC_DATA_LENGTH]={0};
//u8 volatile xbc_down[3];
u8 volatile xbc_up[3];
u8 volatile timer_flag =1;

void xbc_init(u8 lcd_on){

	SPI_InitTypeDef SPI_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2 , ENABLE );
	xbc_gpio_init();
	
	SPI_StructInit(&SPI_InitStructure );
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // max speed is SPI_BaudRatePrescaler_8, faster would not function in use long cable
	SPI_I2S_DeInit(XBC_SPI); 
	SPI_Cmd(XBC_SPI, DISABLE);  
	SPI_Init(XBC_SPI, &SPI_InitStructure);
	SPI_SSOutputCmd( XBC_SPI , ENABLE );
	SPI_CalculateCRC(XBC_SPI, DISABLE); 
	SPI_StructInit( &SPI_InitStructure );
	SPI_Cmd(XBC_SPI, ENABLE);
	
	xbc_timer_init();

	if (!lcd_on)
		lcd_extend_enable = 3;

}

void xbc_gpio_init(void){
	GPIO_InitTypeDef XBC_SPI_GPIO;

	/* Configure SPI1 pins: SCK , MISO and MOSI
	 * Configure SCK and MOSI pins as Alternate Function Push Pull,
	 * MISO pin as Input Floating
	 */
	XBC_SPI_GPIO.GPIO_Pin = XBC_SPI_SCK | XBC_SPI_MISO;
	XBC_SPI_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	XBC_SPI_GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(XBC_SPI_PORT, &XBC_SPI_GPIO);

	XBC_SPI_GPIO.GPIO_Pin = XBC_SPI_MOSI;
	XBC_SPI_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	XBC_SPI_GPIO.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(XBC_SPI_PORT, &XBC_SPI_GPIO);
	
	/* Configure SPI1 pins: NSS (chip select)*/
	XBC_SPI_GPIO.GPIO_Pin = XBC_SPI_NSS;
	XBC_SPI_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	XBC_SPI_GPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(XBC_SPI_PORT, &XBC_SPI_GPIO);
	
	XBC_SPI_GPIO.GPIO_Mode = GPIO_Mode_IPD;
	XBC_SPI_GPIO.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOD, &XBC_SPI_GPIO);
	
	GPIO_SetBits( XBC_SPI_PORT , XBC_SPI_NSS );
}

u8 get_xbc_mode(){
	return xbc_mode;
}

void xbc_check_flag (void) 
{

}


u8 xbc_update(){
	
/*
	*Call period:
		20ms
	*Return:
		0: error, no right data
		1: right data
		2: maybe disconnect
 	*rebuild received data in a easy way to use
*/
	u8 ck_flag = 1;
	u8 first_check_bit = 0x14;
	u8 last_check_bit = 0x15;
	
/*		for(i=0;i<16;i++){
			printf("%xH ",xbc_data[i]);
		}
			printf("\r\n");
			printf("%lx\r\n",xbc_digital);
*/
		if (!running) // enable xbc
			running = 1;

		if (xbc_data[0] != first_check_bit || xbc_data[14] !=last_check_bit){ //check bit start & end bit
			error_xbc_data_cnt++;
			ck_flag = 0;
			//disconnect_cnt = 0;
		}

		if (ck_flag && xbc_mode == CONNECTED){
			old_xbc_digital = xbc_digital;
			xbc_digital = (xbc_data[13] << 16) + (xbc_data[2] << 8) + xbc_data[1];
			//xbc_digital = xbc_digital & 0x00ffff;
			xbc_joy[XBC_LT] = (u8)xbc_data[3]; //lt max:255
			xbc_joy[XBC_RT] = (u8)xbc_data[4]; //rt max:255
			xbc_joy[XBC_LX] = ((xbc_data[6]<<8) + xbc_data[5]); // lx: -3xxxx to 3xxxx
			xbc_joy[XBC_LY] = ((xbc_data[8]<<8) + xbc_data[7]); // ly: -3xxxx to 3xxxx
			xbc_joy[XBC_RX] = ((xbc_data[10]<<8) + xbc_data[9]); // rx: -3xxxx to 3xxxx
			xbc_joy[XBC_RY] = ((xbc_data[12]<<8) + xbc_data[11]); // ry: -3xxxx to 3xxxx

			/*
			xbc_joy[XBC_LX] = xbc_joy[XBC_LX] > 8000 ? xbc_joy[XBC_LX] : xbc_joy[XBC_LX] < -8000 ? xbc_joy[XBC_LX]:0;
			xbc_joy[XBC_LY] = xbc_joy[XBC_LY] > 8000 ? xbc_joy[XBC_LY] : xbc_joy[XBC_LY] < -8000 ? xbc_joy[XBC_LY]:0;
			xbc_joy[XBC_RX] = xbc_joy[XBC_RX] > 8000 ? xbc_joy[XBC_RX] : xbc_joy[XBC_RX] < -8000 ? xbc_joy[XBC_RX]:0;
			xbc_joy[XBC_RY] = xbc_joy[XBC_RY] > 8000 ? xbc_joy[XBC_RY] : xbc_joy[XBC_RY] < -8000 ? xbc_joy[XBC_RY]:0;
			*/
			xbc_press = xbc_digital & ( xbc_digital ^ old_xbc_digital);
			xbc_release = old_xbc_digital & ( xbc_digital ^ old_xbc_digital);
			error_xbc_data_cnt = 0;
			return 1; //return vaild
		}
		else if (error_xbc_data_cnt >50 && !ck_flag){
			xbc_digital = 0;
			xbc_joy[XBC_LT] = 0; //lt max:255
			xbc_joy[XBC_RT] = 0; //rt max:255
			xbc_joy[XBC_LX] = 0;
			xbc_joy[XBC_LY] = 0;
			xbc_joy[XBC_RX] = 0;
			xbc_joy[XBC_RY] = 0;
			xbc_press = 0;
			xbc_release = 0;
			xbc_channel = 1;
			error_xbc_data_cnt = 0;
		}

		if (xbc_press & XBC_XBOX){
			if (xbc_channel++ > 4)
				xbc_channel = 1;
		}

	return 0;
}


void xbc_tft_transmit(void){

	switch( xbc_state  ){
		case 0:
			//xbc_chip_select();
			SPI_I2S_SendData(XBC_SPI , XBC_HELLO);
			xbc_state ++;
			break;
		case 1:
			if (xbc_tft_section%2 == 0)
				SPI_I2S_SendData(XBC_SPI , COMMAND_TFT_CL); //send color
			else
				SPI_I2S_SendData(XBC_SPI , COMMAND_TFT);
			xbc_state ++;
			break;
		case 2:				
			if (spi_data == 0xFF || spi_data == 0x00) 
			{
				xbc_state = 0;
				if(  xbc_error_count ++ > 50 ){
				//	xbc_chip_deselect();
					transmit_state=0;
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
					  -timer is 0.1ms has one trigger, so 88 bytes need at least 10ms to finish
			xbc_tft_y:line number
			xbc_tft_x_pos:x position of a byte on a line		
			***TRY TO THINK OF THE MATHS***
					---Variable Definition---*/

			//tft_done++;
			xbc_tft_x_pos=(xbc_state-3)%17;
			if(xbc_tft_x_pos ==0 ){
				//xbc_tft_y=xbc_tft_section%2?(xbc_state-3)/17:(xbc_state-3)/17+5; //13-1-2013
				xbc_tft_y=xbc_tft_section/2?(xbc_state-3)/17:(xbc_state-3)/17+5;
				SPI_I2S_SendData( XBC_SPI , XBC_TFT_LINE | xbc_tft_y );
			//	printf("\r\nL:%d\r\n",xbc_tft_y);
			}
			else{
				if (xbc_tft_section%2 == 0){ //send (color / data)
					SPI_I2S_SendData(XBC_SPI , text_bg_color_prev[xbc_tft_x_pos-1][xbc_tft_y] );
				}
				else{
					SPI_I2S_SendData(XBC_SPI , text[xbc_tft_x_pos-1][xbc_tft_y] );

				}
			//	printf("W:%x ",text[xbc_tft_y][xbc_tft_x_pos-1]);
			}
			xbc_state++;

			if(xbc_state > 88 && (xbc_tft_section == 0 || xbc_tft_section == 1)){ //xbc_tft_section: 0,2 send text and 1,3 send color
				xbc_state = 0;
				transmit_state=3;
				xbc_tft_section++;
			}
			else if(xbc_state > 88 && (xbc_tft_section == 2 || xbc_tft_section == 3)){
				xbc_state = 0;
				transmit_state=3;
				if (xbc_tft_section++ > 3)
					xbc_tft_section = 0;
			}
			else if(xbc_state > 88 && (xbc_tft_section > 3)){ //invaild state and value
				xbc_state = 0;
				xbc_tft_section = 0;
				transmit_state=0;
			}
			break;

/********************************************************************/
	}
}

void get_xbc_data(void){
	static u8 spi_data_length = 0;
	if( xbc_state != 0 )
		spi_data = SPI_I2S_ReceiveData(XBC_SPI);
	//transmit_state=1;
	switch(xbc_state ){
		case 0:

			SPI_I2S_SendData(XBC_SPI , XBC_HELLO);
			xbc_state ++;
			break;
		case 1:
			if (xbc_tft_section%2 == 0) //a var use to check tft send (color/data)
				SPI_I2S_SendData(XBC_SPI , COMMAND_XBC); //if send color
			else
				SPI_I2S_SendData(XBC_SPI , COMMAND_XBC); //if send
			xbc_state ++;
			break;
		case 2:
			if (spi_data == 0xFF || spi_data == 0x00)
			{
				xbc_state = 0;
				if(xbc_error_count ++ > 50 ){
					transmit_state=0;
					xbc_error_count = 0;
				}
			}
			else{
				xbc_error_count = 0;
				SPI_I2S_SendData(XBC_SPI, XBC_COMFIRMED);
			//	spi_data_length = (spi_data & 0x0F) << 1;
				if (spi_data == XBOX_CONTROLLER){

					is_xbc_controller=1;
					spi_data_length=15;
				}
				xbc_state ++;
				xbc_data[0] = 0;
				xbc_data[14] = 0;
			}

			break;

		default: // state = 3 ++

				if(spi_data_length--){
					SPI_I2S_SendData( XBC_SPI , XBC_COMFIRMED);
					if( xbc_state >= 4 )
						xbc_data[xbc_state - 4] = spi_data;
					xbc_state ++;
				}
				else{
					if( xbc_state >= 4 )
						xbc_data[xbc_state - 4] = spi_data;
					xbc_state = 0;

					transmit_state=lcd_extend_enable;
				//	transmit_state=2;
				//	transmit_state=3;
				}
		break;

	}
}

/***
 * transmit way: rx(xbc) -> tx(tft) -> rx(xbc) -> tx(tft) -> rx(xbc) -> tx(tft) -> rx(xbc) -> tx(tft)
 *	                      line:0-4 text        line:0-4 color        line:5-9 text        line:5-9 color
 *                 3ms        15ms       3ms        15ms       3ms      15ms         3ms       15ms
***/
void TIM7_IRQHandler(void){

	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);

	if (!running)
		return;

	switch(transmit_state){
		case 0:
			transmit_state=1;
			transmit_mode=tft_tx_mode;
			transmit_mode=xbc_rx_mode;
			break;

		case 1: //waiting xbc trigger
			if (transmit_mode == xbc_rx_mode && !GPIO_ReadInputDataBit(XBC_SPI_PORT,XBC_NSS)){
				get_xbc_data();
				disconnect_cnt = 0;
				xbc_mode = 1;
			}
			else if (transmit_mode == tft_tx_mode && !GPIO_ReadInputDataBit(XBC_SPI_PORT,XBC_NSS)){ //send tft to xbc
				xbc_tft_transmit();
				disconnect_cnt = 0;
				xbc_mode = 1;
			}
			else{
				//transmit_state++;
				xbc_counter = 0;
				if (disconnect_cnt < 650)
					disconnect_cnt++;
				else if (disconnect_cnt > 649)
					xbc_mode = 0;

			}
		break;

		case 2:

			if( xbc_counter ++ > 30 ){
				xbc_counter = 0;
				transmit_mode = tft_tx_mode;
			//	transmit_mode = xbc_rx_mode;
				transmit_state=1;
			}
		break;

		case 3:

			if( xbc_counter ++ > 30 ){
				xbc_counter = 0;
				transmit_mode = xbc_rx_mode;
				transmit_state=1;
			}
			break;

		case 4: //

			if( xbc_counter ++ > 30 ){
				xbc_counter = 0;
				transmit_mode = tft_tx_mode;
				transmit_state=1;
			}

		break;

		case 5: //for test tft mode

			if( xbc_counter ++ > 30 ){
				xbc_counter = 0;
				transmit_mode = tft_tx_mode;
				transmit_state=1;
			}

		break;

		default:
			transmit_state = 0;
			break;
	}
}

void xbc_timer_init(void){
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);
	TIM7->ARR = 8 * 2 + 14;
	TIM7->PSC = 256 - 1 ;
	TIM7->EGR = 1;
	TIM7->SR = 0;
	TIM7->DIER = 1;
	TIM7->CR1 = 1;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn; 
	NVIC_Init(&NVIC_InitStructure);
}

void disable_xbc(void)
{
	running = 0;
}

void enable_xbc(void)
{
	running = 1;
}

void xbc_test_program(void)
{
	u16 ticks_img = 0;
	u8	pressed_cnt = 0;
	u8  pre_pressed_cnt = 0;
	u16 press_times = 0;
	u8 xbc_test_ko = 0;
	u32 pre_xbc_press = 0;
	
	
	tft_prints(0,0,"XBOX CONTROLLER");
	tft_prints(0,1,"testing program");
	tft_prints(0,2,"PRESS [START]");
	
	while(!xbc_test_ko)
	{
		if (ticks_img != get_ticks()) {
			if (ticks_img % 20 == 0) {
				pre_pressed_cnt = pressed_cnt;
				pre_xbc_press = xbc_press;
				xbc_update();
				switch (pressed_cnt)
				{
					case 0:
						if (xbc_press & XBC_START) {
							tft_clear();
							tft_prints(0,0,"PRESS [UP]");
							pressed_cnt++;
						}
					break;

					case 1:
						if (xbc_press & XBC_UP) {
							tft_prints(0,0,"PRESS [DOWN]   ");
							pressed_cnt++;
						}
					break;

					case 2:
						if (xbc_press & XBC_DOWN) {
							tft_prints(0,0,"PRESS [LEFT]   ");
							pressed_cnt++;
						}
					break;

					case 3:
						if (xbc_press & XBC_LEFT) {
							tft_prints(0,0,"PRESS [RIGHT]   ");
							pressed_cnt++;
						}
					break;

					case 4:
						if (xbc_press & XBC_RIGHT) {
							tft_prints(0,0,"PRESS [BACK]   ");
							pressed_cnt++;
						}
					break;

					case 5:
						if (xbc_press & XBC_BACK) {
							tft_prints(0,0,"PRESS [A]     ");
							pressed_cnt++;
						}
					break;

					case 6:
						if (xbc_press & XBC_A)
						{
							tft_prints(0,0,"PRESS [B]    ");
							pressed_cnt++;
						}
					break;

					case 7:
						if (xbc_press & XBC_B)
						{
							tft_prints(0,0,"PRESS [X]    ");
							pressed_cnt++;
						}
					break;

					case 8:
						if (xbc_press & XBC_X) {
							tft_prints(0,0,"PRESS [Y]     ");
							pressed_cnt++;
						}
					break;

					case 9:
						if (xbc_press & XBC_Y) {
							tft_prints(0,0,"PRESS [LB ]   ");
							pressed_cnt++;
						}
					break;

					case 10:
						if (xbc_press & XBC_LB) {
							tft_prints(0,0,"PRESS [RB ]   ");
							pressed_cnt++;
						}
					break;

					case 11:
						if (xbc_press & XBC_RB) {
							tft_prints(0,0,"PRESS [XBOX  ]   ");
							pressed_cnt++;
						}
					break;

					case 12:
						if (xbc_press & XBC_XBOX) {
							tft_prints(0,0,"[L BUTTON 1]");
							pressed_cnt++;
						}
					break;

					case 13:
						if (xbc_press & L_BUT1) {
							tft_prints(0,0,"[L BUTTON 2]");
							pressed_cnt++;
						}
					break;

					case 14:
						if (xbc_press & L_BUT2) {
							tft_prints(0,0,"[L BUTTON 3]");
							pressed_cnt++;
						}
					break;

					case 15:
						if (xbc_press & L_BUT3) {
							tft_prints(0,0,"[R BUTTON 1]");
							pressed_cnt++;
						}
					break;


					case 16:
						if (xbc_press & R_BUT1) {
							tft_prints(0,0,"[R BUTTON 2]");
							pressed_cnt++;
						}
					break;

					case 17:
						if (xbc_press & R_BUT2) {
							tft_prints(0,0,"[R BUTTON 3]");
							pressed_cnt++;
						}
					break;

					case 18:
						if (xbc_press & R_BUT3) {
							tft_prints(0,0,"KEY TEST OK");
							tft_prints(0,0,"press [BACK] exit");
							pressed_cnt++;
						}
					break;

					case 19:
						xbc_test_ko = 1;
					break;

					default:
						xbc_test_ko = 1;
					break;
				}
				
				if (xbc_press != pre_xbc_press && xbc_press != 0) {
					press_times++;
				}

				tft_prints(0,3,"LT:%3ld RT:%3ld",xbc_joy[XBC_LT],xbc_joy[XBC_RT]);
				tft_prints(0,4,"LX:%5ld",xbc_joy[XBC_LX]);
				tft_prints(0,5,"LY:%5ld",xbc_joy[XBC_LY]);
				tft_prints(0,6,"RX:%5ld",xbc_joy[XBC_RX]);
				tft_prints(0,7,"RY:%5ld",xbc_joy[XBC_RY]);
				tft_prints(0,8,"press_cnt:%3d",press_times); //normally press once count up 1
				tft_prints(15,9,"%d",ticks_img);
				tft_update();
			}
		}
	}
}
