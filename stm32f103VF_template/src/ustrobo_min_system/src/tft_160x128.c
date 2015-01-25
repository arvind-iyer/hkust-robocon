#include "tft_160x128.h"
#include "lcd_font.h"
#include "ticks.h"

//private data
u16 _bg_color = 0 , _text_color = 0 , _special_color=0;
u8 tft_orientation = 0, tft_enabled = 1;
char text[MAX_LINE][MAX_WORD];
char pre_text[MAX_LINE][MAX_WORD];
u16 text_color[MAX_LINE][MAX_WORD];
u16 pre_text_color[MAX_LINE][MAX_WORD];
u16 bg_color[MAX_LINE][MAX_WORD];
u16 pre_bg_color[MAX_LINE][MAX_WORD];
u16 print_pos = 0;
char u16_print_buffer[6];
char u8_print_buffer[4];

void tft_set_bg_color(u16 bg_color);				//set background color
void tft_set_text_color(u16 text_color);			//set text color
void tft_set_special_color( u16 special_color );//set special color

void tft_spi_init(void)
{
   SPI_InitTypeDef   	SPI_InitStructure;
   GPIO_InitTypeDef 	GPIO_InitStructure;

   /* Enable TFT_SPI and GPIO clocks */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_RST | RCC_APB2Periph_GPIO_DC | 
					                RCC_APB2Periph_GPIO_CS | RCC_APB2Periph_GPIO_CLK | 
	                        RCC_APB2Periph_GPIO_MOSI | RCC_APB2Periph_AFIO , ENABLE);
	 RCC_APB2PeriphClockCmd(TFT_SPI_RCC , ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
   /** Configure TFT_SPI Pin: SCK and MOSI 
	   * Should in GPIO_Mode_AF_PP mode 
	 **/
   GPIO_InitStructure.GPIO_Pin =  TFT_CLK_PIN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(TFT_CLK_PORT, &GPIO_InitStructure);
	
	 GPIO_InitStructure.GPIO_Pin =  TFT_MOSI_PIN;
   GPIO_Init(TFT_MOSI_PORT, &GPIO_InitStructure);
	
	/** Configure TFT_GPIO Pin: RST, DC and CS 
	   * Should in GPIO_Mode_Out_PP mode 
	 **/
	 GPIO_InitStructure.GPIO_Pin =  TFT_RST_PIN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
   GPIO_Init(TFT_RST_PORT, &GPIO_InitStructure);
	
	 GPIO_InitStructure.GPIO_Pin =  TFT_DC_PIN;
	 GPIO_Init(TFT_DC_PORT, &GPIO_InitStructure);
	 
	 GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_CS;
	 GPIO_Init(GPIO_CS, &GPIO_InitStructure);
  
   /* TFT_SPI configuration */
   SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
   SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
   SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
   SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
   SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
   SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
   SPI_Init(TFT_SPI, &SPI_InitStructure);
   /* Enable TFT_SPI   */
   SPI_Cmd(TFT_SPI, ENABLE);
   SPI_CalculateCRC( TFT_SPI , DISABLE );		// Disable the CRC checking
   SPI_SSOutputCmd( TFT_SPI , DISABLE );
	 
}

void tft_write_data(u8 data)
{
	u8 i = 2;
	GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
	GPIO_SetBits(TFT_DC_PORT, TFT_DC_PIN);

  /* Send byte through the TFT_SPI peripheral */
   SPI_I2S_SendData(TFT_SPI, data);
	while(i)
		i--;
}

void tft_write_command(u8 command)
{
	u8 i = 2;
	GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
	GPIO_ResetBits(TFT_DC_PORT, TFT_DC_PIN);
	
  /* Send byte through the TFT_SPI peripheral */
   SPI_I2S_SendData(TFT_SPI, command);
	while(i)
		i--;
}

void tft_config(void)
{
	tft_write_command(0x01);   //Sofeware setting
	_delay_ms(10);
	tft_write_command(0x11);//Sleep out
	_delay_ms(120);
	
	//ST7735R Frame Rate
	tft_write_command(0xB1);
	tft_write_data(0x01);
	tft_write_data(0x2C);
	tft_write_data(0x2D);
	tft_write_command(0xB2);
	tft_write_data(0x01);
	tft_write_data(0x2C);
	tft_write_data(0x2D);
	tft_write_command(0xB3);
	tft_write_data(0x01);
	tft_write_data(0x2C);
	tft_write_data(0x2D);
	tft_write_data(0x01);
	tft_write_data(0x2C);
	tft_write_data(0x2D);
	//------------------------------------End ST7735R Frame Rate-----------------------------------------//
	tft_write_command(0xB4);//Column inversion
	tft_write_data(0x07);
	//------------------------------------ST7735R Power Sequence-----------------------------------------//
	tft_write_command(0xC0);
	tft_write_data(0xA2);
	tft_write_data(0x02);
	tft_write_data(0x84);
	tft_write_command(0xC1);
	tft_write_data(0xC5);
	tft_write_command(0xC2);
	tft_write_data(0x0A);
	tft_write_data(0x00);
	tft_write_command(0xC3);
	tft_write_data(0x8A);
	tft_write_data(0x2A);
	tft_write_command(0xC4);
	tft_write_data(0x8A);
	tft_write_data(0xEE);
	//---------------------------------End ST7735R Power Sequence-------------------------------------//
	tft_write_command(0xC5);//VCOM
	tft_write_data(0x0E);
	tft_write_command(0x36);//MX, MY, RGB mode
	tft_write_data(0xC8);
	//------------------------------------ST7735R Gamma Sequence-----------------------------------------//
	tft_write_command(0xe0);
	tft_write_data(0x02);
	tft_write_data(0x1c);
	tft_write_data(0x07);
	tft_write_data(0x12);
	tft_write_data(0x37);
	tft_write_data(0x32);
	tft_write_data(0x29);
	tft_write_data(0x2d);
	tft_write_data(0x29);
	tft_write_data(0x25);
	tft_write_data(0x2b);
	tft_write_data(0x39);
	tft_write_data(0x00);
	tft_write_data(0x01);
	tft_write_data(0x03);
	tft_write_data(0x10);
	tft_write_command(0xe1);
	tft_write_data(0x03);
	tft_write_data(0x1d);
	tft_write_data(0x07);
	tft_write_data(0x06);
	tft_write_data(0x2e);
	tft_write_data(0x2c);
	tft_write_data(0x29);
	tft_write_data(0x2d);
	tft_write_data(0x2e);
	tft_write_data(0x2e);
	tft_write_data(0x37);
	tft_write_data(0x3f);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x02);
	tft_write_data(0x10);
	tft_write_command(0x2A);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x7f);

	tft_write_command(0x2B);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x9f);
	//------------------------------------End ST7735R Gamma Sequence-----------------------------------------//

	tft_write_command(0x3A);
	tft_write_data(0x05);  //0x05 = 16bits, 0x03 = 12bits, 0x06 = 18bits
	tft_write_command(0x29);//Display on

	_delay_ms(10);
}


void tft_reset(void)
{
 	GPIO_ResetBits(TFT_RST_PORT, TFT_RST_PIN);
	_delay_ms(100);
	GPIO_SetBits(TFT_RST_PORT, TFT_RST_PIN);
	_delay_ms(100);
}

void tft_fill_color(u16 color)
{
	u16 i=20480;					//160*128
	
	/* dummy data to reset status of TFT */
	
	tft_write_command(0x2a);		// Column addr set
	tft_write_data(0x00);
	tft_write_data(0x00); 				// X START
	tft_write_data(0x00);
	tft_write_data(0x7f); 			// X END

	tft_write_command(0x2b);		// Row addr set
	tft_write_data(0x00);
	tft_write_data(0x00);				// Y START
	tft_write_data(0x00);
	tft_write_data(0x9f);			// Y END

	tft_write_command(0x2c); 		// write to RAM
	
	while(--i){
		tft_write_data(_bg_color>>8);
		tft_write_data((_bg_color));
	}		
}

void tft_set_bg_color(u16 bg_color)
{
	_bg_color = bg_color ;
}

u16 get_curr_bg_color(){
	return _bg_color;
}

void tft_set_text_color( u16 text_color ){
	_text_color = text_color;
}

void tft_set_special_color( u16 special_color ){
	_special_color = special_color;
}

// Horizontal: 0-15
// Vertical: 0-9
void tft_prints(u8 x, u8 y, const char * pstr, ...)
{
	u8 buf[256], is_special = 0;
	u8* fp=NULL;
	va_list arglist;	
	va_start(arglist, pstr);
	vsprintf((char*)buf, (const char*)pstr, arglist);
	va_end(arglist);

	fp = buf;
	while (*fp)	{
		if(*fp == '['){
			is_special = 1;
			fp++;
		}
		else if(*fp == ']'){
			is_special = 0;
			fp++;
		}		
		else if(*fp == '\r' || *fp == '\n'){		  				 
			fp++;
		}
		else{
			text[y][x] = *fp++;
			text_color[y][x] = is_special ? _special_color : _text_color;
			bg_color[y][x] = _bg_color;
			if(x >= MAX_WORD-1)	{
				x = 0;
				y++;
			}
			else{
				x++;
			}
			if( y >= MAX_LINE-1 )
				y = 0;
		}
	}
	is_special = 0;
}

void lcd_print_queue_enqueue(u8,u8,u8,u16,u16);

void tft_string_prints(u8 x, u8 y, const char* a){
	u8* fp = (u8*)a;
	while (*fp)	{
		text[y][x] = *fp++;
		text_color[y][x] = _text_color;
		bg_color[y][x] = _bg_color;
		if(x > MAX_WORD-1)	{
			x = 0;
			y++;
		}
		else{
			x++;
		}
		if( y > MAX_LINE-1 )
			y = 0;
	}
}

void tft_u16_prints(u8 x, u8 y, u16 a){
	u8 i = 0; u8 temp = 0;
	temp = a/10000;
	if(temp != 0){
		u16_print_buffer[i] = temp+48;
		a = a%10000;
		++i;
	}
	temp = a/1000;
	if(temp != 0 || i != 0){
		u16_print_buffer[i] = temp+48;
		a = a%1000;
		++i;
	}
	temp = a/100;
	if(temp != 0 || i != 0){
		u16_print_buffer[i] = temp+48;
		a = a%100;
		++i;
	}
	temp = a/10;
	if(temp != 0 || i != 0){
		u16_print_buffer[i] = temp+48;
		a = a%10;
		++i;
	}
	u16_print_buffer[i] = a+48;
	u16_print_buffer[i+1] = 0;
	tft_string_prints(x,y,u16_print_buffer);
}

void tft_u8_prints(u8 x, u8 y, u8 a){
	u8 i = 0; u8 temp = 0;
	temp = a/100;
	if(temp != 0){
		u8_print_buffer[i] = temp+48;
		a = a%100;
		++i;
	}
	temp = a/10;
	if(temp != 0 || i != 0){
		u8_print_buffer[i] = temp+48;
		a = a%10;
		++i;
	}
	u8_print_buffer[i] = a+48;
	u8_print_buffer[i+1] = 0;
	tft_string_prints(x,y,u8_print_buffer);
}

void tft_update(void){
	u8 x , y;
	if (!tft_enabled)
		return;
	for( y = 0 ; y < MAX_LINE ; y ++ ){
		for( x = 0 ; x < MAX_WORD ; x ++ ){
			if ( pre_text[y][x] != text[y][x] || pre_text_color[y][x] != text_color[y][x] || pre_bg_color[y][x] != bg_color[y][x]) {
				pre_text[y][x] = text[y][x];
				pre_text_color[y][x] = text_color[y][x];
				pre_bg_color[y][x] = bg_color[y][x];
				lcd_print_queue_enqueue(text[y][x],x,y,text_color[y][x],bg_color[y][x]);
			}
		}
	}
}

void tft_force_clear(void) {
	u8 x,y;
	for (y = 0; y < MAX_LINE; y++) {
		for (x = 0; x < MAX_WORD; x++) {
			text[y][x] = ' ';
			text_color[y][x]=_text_color;
			bg_color[y][x]=_bg_color;
		}
	}
	tft_fill_color(_bg_color);
}

void tft_init(u8 orientation, u16 local_bg_color, u16 normal_color, u16 special_color)
{
	tft_spi_init();
	tft_reset();
	tft_config();
  tft_write_command(0x2C);
	tft_set_bg_color(local_bg_color);
	tft_set_text_color(normal_color);
	tft_set_special_color(special_color);
	tft_fill_color(_bg_color);
	tft_orientation = orientation;

	tft_clear();
	GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
}

void tft_clear_line(u8 line ){
	u8 x;
	for( x = 0 ; x < MAX_WORD ; x ++ ){
		text[line][x] = ' ';
		text_color[line][x]=_text_color;
		bg_color[line][x]=_bg_color;
	}
}

void tft_clear(void ){
	u8 y;
	for( y=0; y < MAX_LINE; y++)
		tft_clear_line(y);
}

void tft_enable(void) {
	tft_enabled = 1;
}

void tft_disable(void) {
	tft_enabled = 0;
}

u8 lcd_position_data[11][2] = {
	//set col position
	{1,0x2a},
	{0,0x00},
	{0,0x00},
	{0,0x00},
	{0,0x07},
	
	//set row position
	{1,0x2b},
	{0,0x00},
	{0,0x00},
	{0,0x00},
	{0,0x0F},
	
	//write to RAM
	{1,0x2c},
};

u16 lcd_char_print_counter = 0;
u8 curr_lcd_char_col = 0, curr_lcd_char_row = 0, curr_lcd_char_hl = 0, curr_lcd_pixel = 0;
u8* curr_printing_char_pointer = (u8*)(ascii_8x16);

u8 lcd_print_queue[QUEUE_SIZE][3];
u16 lcd_print_queue_color[QUEUE_SIZE];
u16 lcd_print_queue_bg_color[QUEUE_SIZE];
u16 lcd_print_queue_start = 0, lcd_print_queue_end = 0;
u16 text_color_for_once = 0, bg_color_for_once = 0;

u16 lcd_print_queue_start_plus(){
	u16 temp = lcd_print_queue_start + 1;
	if(temp == QUEUE_SIZE)
		temp = 0;
	return temp;
}

u16 lcd_print_queue_end_plus(){
	u16 temp = lcd_print_queue_end + 1;
	if(temp == QUEUE_SIZE)
		temp = 0;
	return temp;
}

u8 lcd_queue_empty(){
	if(lcd_print_queue_start == lcd_print_queue_end)
		return 1;
	return 0;
}

void lcd_print_queue_enqueue(u8 a, u8 b, u8 c, u16 color, u16 bg_color){
	
	lcd_print_queue[lcd_print_queue_end][0] = a;
	lcd_print_queue[lcd_print_queue_end][1] = b;
	lcd_print_queue[lcd_print_queue_end][2] = c;
	lcd_print_queue_color[lcd_print_queue_end] = color;
	lcd_print_queue_bg_color[lcd_print_queue_end] = bg_color;
	lcd_print_queue_end = lcd_print_queue_end_plus();
	
}

u8 master_print_flag = 0;

void lcd_update_char(){
	u8 i = 0; 
	for(i = 0; i < UPDATE_SPEED_LEVEL; i++){
		if(lcd_char_print_counter < 11 && !master_print_flag){
			++i;
			GPIO_WriteBit(TFT_DC_PORT, TFT_DC_PIN, (BitAction)(!lcd_position_data[lcd_char_print_counter][0]));
			SPI_I2S_SendData(TFT_SPI, lcd_position_data[lcd_char_print_counter][1]);
				
		}else if(lcd_char_print_counter == 267){
			if(!lcd_queue_empty()){
				curr_printing_char_pointer = (u8*)(ascii_8x16) + ((lcd_print_queue[lcd_print_queue_start][0]-32)<<4);
				lcd_position_data[2][1] = lcd_print_queue[lcd_print_queue_start][1]<<3;
				lcd_position_data[4][1] = (lcd_print_queue[lcd_print_queue_start][1]<<3) + 7;
				lcd_position_data[7][1] = lcd_print_queue[lcd_print_queue_start][2]<<4;
				lcd_position_data[9][1] = (lcd_print_queue[lcd_print_queue_start][2]<<4) + 15;
				text_color_for_once = lcd_print_queue_color[lcd_print_queue_start];
				bg_color_for_once = lcd_print_queue_bg_color[lcd_print_queue_start];
				lcd_print_queue_start = lcd_print_queue_start_plus();		
				master_print_flag = 0;				
			}else{
				master_print_flag = 1;
			}
		}else if(!master_print_flag){
			GPIO_WriteBit(TFT_DC_PORT, TFT_DC_PIN, (BitAction)1);
			if(curr_lcd_char_hl == 0){
				curr_lcd_pixel = *(curr_printing_char_pointer + curr_lcd_char_row) >> ( 7 - curr_lcd_char_col);
				if(curr_lcd_pixel & 0x01){
					SPI_I2S_SendData(TFT_SPI, text_color_for_once>>8);
				}else{
					SPI_I2S_SendData(TFT_SPI, bg_color_for_once>>8);
				}
				curr_lcd_char_hl = 1;
			}else{
				if(curr_lcd_pixel & 0x01){
					SPI_I2S_SendData(TFT_SPI, text_color_for_once);
				}else{
					SPI_I2S_SendData(TFT_SPI, bg_color_for_once);
				}
				curr_lcd_char_hl = 0;

				++curr_lcd_char_col;
				if(curr_lcd_char_col == 8){
					curr_lcd_char_col = 0;
					++curr_lcd_char_row;
					if(curr_lcd_char_row == 16){
						curr_lcd_char_row = 0;
					}
				}
			}
		}
		++lcd_char_print_counter;
		if(lcd_char_print_counter == 268)
			lcd_char_print_counter = 0;
	}
}

void print_picture(u8* a){
	u16 i = 0;					//160*128 * 1.5
	
	/* dummy data to reset status of TFT */
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x00);
	
	tft_write_command(0x2a);		// Column addr set
	tft_write_data(0x00);
	tft_write_data(0x00); 				// X START
	tft_write_data(0x00);
	tft_write_data(0x7f); 			// X END

	tft_write_command(0x2b);		// Row addr set
	tft_write_data(0x00);
	tft_write_data(0x00);				// Y START
	tft_write_data(0x00);
	tft_write_data(0x9f);			// Y END

	tft_write_command(0x2c); 		// write to RAM
	
	while(i < 40960){
		tft_write_data(*a);
		++a;
		++i;
	}	
}

void tft_master_clear(){
	u8 x;
	u8 y;
	for( y=0; y < MAX_LINE; y++){
		for( x = 0 ; x < MAX_WORD ; x ++ ){
			text[y][x] = ' ';
			text_color[y][x]=_text_color;
			bg_color[y][x]=_bg_color;
			pre_text[y][x] = ' ';
			pre_text_color[y][x]=_text_color;
			pre_bg_color[y][x]=_bg_color;
		}
	}
}

//u8 SD_picture_buffer[10240];
//void print_picture_from_SD(u32 address){
//	u16 i = 0;					//160*128 * 1.5
//	u8* temp_adress = (u8*)SD_picture_buffer;
//	
//	/* dummy data to reset status of TFT */
//	tft_write_data(0x00);
//	tft_write_data(0x00);
//	tft_write_data(0x00);
//	tft_write_data(0x00);
//	tft_write_data(0x00);
//	tft_write_data(0x00);
//	tft_write_data(0x00);
//	tft_write_data(0x00);
//	
//	tft_write_command(0x2a);		// Column addr set
//	tft_write_data(0x00);
//	tft_write_data(0x00); 				// X START
//	tft_write_data(0x00);
//	tft_write_data(0x7f); 			// X END

//	tft_write_command(0x2b);		// Row addr set
//	tft_write_data(0x00);
//	tft_write_data(0x00);				// Y START
//	tft_write_data(0x00);
//	tft_write_data(0x9f);			// Y END

//	tft_write_command(0x2c); 		// write to RAM
//	
//	//SD_WaitReadOperation();
//  while(SD_GetStatus() != SD_TRANSFER_OK);
//	SD_ReadMultiBlocks(SD_picture_buffer, address+i, 512, 20);
//	SD_WaitReadOperation();
//  while(SD_GetStatus() != SD_TRANSFER_OK);
//	while(i < 10240){
//		tft_write_data(*temp_adress);
//		++temp_adress;
//		++i;
//	}
//	
//	temp_adress = (u8*)SD_picture_buffer;
//	SD_ReadMultiBlocks(SD_picture_buffer, address+i, 512, 20);
//	SD_WaitReadOperation();
//  while(SD_GetStatus() != SD_TRANSFER_OK);
//	while(i < 20480){
//		tft_write_data(*temp_adress);
//		++temp_adress;
//		++i;
//	}
//	
//	temp_adress = (u8*)SD_picture_buffer;
//	SD_ReadMultiBlocks(SD_picture_buffer, address+i, 512, 20);
//	SD_WaitReadOperation();
//  while(SD_GetStatus() != SD_TRANSFER_OK);
//	while(i < 30720){
//		tft_write_data(*temp_adress);
//		++temp_adress;
//		++i;
//	}
//	
//	temp_adress = (u8*)SD_picture_buffer;
//	SD_ReadMultiBlocks(SD_picture_buffer, address+i, 512, 20);
//	SD_WaitReadOperation();
//  while(SD_GetStatus() != SD_TRANSFER_OK);
//	while(i < 40960){
//		tft_write_data(*temp_adress);
//		++temp_adress;
//		++i;
//	}
//}

