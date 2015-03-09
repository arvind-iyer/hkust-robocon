#include "tft_160x128.h"
#include "lcd_font.h"


/*
 *  **for Xbox controller only**
 */

//private data
u16 _bg_color = 0 , _text_color = 0 , _special_color=0;
u8 tft_orientation = 0, tft_enabled = 1;
char text[MAX_LINE][MAX_WORD];
char pre_text[MAX_LINE][MAX_WORD];
u16 text_color[MAX_LINE][MAX_WORD];
u16 pre_text_color[MAX_LINE][MAX_WORD];
u8 text_bg_color_prev[MAX_LINE][MAX_WORD];
u16 print_pos = 0;

void tft_set_bg_color(u16 bg_color);				//set background color
void tft_set_text_color(u16 text_color);			//set text color
void tft_set_special_color( u16 special_color );//set special color
void tft_set_all_color(void);

void tft_spi_init(void)
{
   SPI_InitTypeDef   	SPI_InitStructure;
   GPIO_InitTypeDef 	GPIO_InitStructure;

   /* Enable TFT_SPI and GPIO clocks */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOB,ENABLE);

   /* Enable GPIOB for RST pin & DC Pin  */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
   GPIO_InitStructure.GPIO_Pin = TFT_RST_PIN | TFT_DC_PIN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);   
   
   /* Configure TFT_SPI Pin: SCK, MISO , MOSI */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SCK | GPIO_Pin_MISO | GPIO_Pin_MOSI;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);   

   /* Configure TFT_SPI Pin: CS */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CS;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
   GPIO_Init(GPIO_CS, &GPIO_InitStructure);
  
   /* TFT_SPI configuration */
   SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
   SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
   SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
   SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
   SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
   SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
   SPI_Init(TFT_SPI, &SPI_InitStructure);
   /* Enable TFT_SPI   */
   SPI_Cmd(TFT_SPI, ENABLE);
   SPI_CalculateCRC( TFT_SPI , DISABLE );		// Disable the CRC checking
   SPI_SSOutputCmd( TFT_SPI , DISABLE );
}

void tft_write_data(u8 data)
{
	GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
	GPIO_SetBits(TFT_DC_PORT, TFT_DC_PIN);

   /* Loop while DR register in not emplty */
   while (SPI_I2S_GetFlagStatus(TFT_SPI, SPI_I2S_FLAG_TXE) == RESET);
   /* Send byte through the TFT_SPI peripheral */
   SPI_I2S_SendData(TFT_SPI, data);
   while (SPI_I2S_GetFlagStatus(TFT_SPI, SPI_I2S_FLAG_RXNE) == RESET );
   SPI_I2S_ReceiveData(TFT_SPI);

   GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
}

void tft_write_command(u8 command)
{
	GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS);
	GPIO_ResetBits(TFT_DC_PORT, TFT_DC_PIN);
	/* Loop while DR register in not emplty */
   while (SPI_I2S_GetFlagStatus(TFT_SPI, SPI_I2S_FLAG_TXE) == RESET);
   /* Send byte through the TFT_SPI peripheral */
   SPI_I2S_SendData(TFT_SPI, command);
   while (SPI_I2S_GetFlagStatus(TFT_SPI, SPI_I2S_FLAG_RXNE) == RESET );
   SPI_I2S_ReceiveData(TFT_SPI);

   GPIO_SetBits(GPIO_CS, GPIO_Pin_CS);
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
	tft_write_data(0x05);  
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

void tft_set_char_pos(u8 x, u8 y){
	tft_write_command(0x2a);		// Column addr set
	tft_write_data(0x00);
	tft_write_data(x*8); 			//X START
	tft_write_data(0x00);
	tft_write_data(x*8+7); 			//X END
	
	tft_write_command(0x2b);		//Row addr set
	tft_write_data(0x00);
	tft_write_data(y*16);			//Y START
	tft_write_data(0x00);
	tft_write_data(y*16+15);		//Y END

	tft_write_command(0x2c); 		// write to RAM
}

void tft_put_char(u8 x, u8 y, u8 ascii, u16 color) {
	u8 i, j, pixel;
	u8 *p = ascii_8x16;
	
	if (ascii > 31 && ascii < 128)
		p += (ascii-32)<<4;
	else
		return;
	
	if (tft_orientation == 2) {
		x = MAX_WORD - 1 - x;
		y = MAX_LINE - 1 - y;
	}	
	tft_set_char_pos(x,y);
	
	for (j = 0; j < WORD_VERTICAL_PIXEL_NO; j++){
		for (i = 0; i < WORD_HORIZONTAL_PIXEL_NO; i++) {				
			if (tft_orientation == 2)
				pixel = *(p + WORD_VERTICAL_PIXEL_NO - 1 - j) >> i;
			else
				pixel = *(p + j) >> (WORD_HORIZONTAL_PIXEL_NO - 1 - i);
		
			if(pixel&0x01){
				tft_write_data(color>>8);
				tft_write_data(color);
			}
			else{				
				tft_write_data(_bg_color>>8);
				tft_write_data(_bg_color);
			}				
		}
	}	
}

void tft_fill_color(u16 color)
{
	u16 i=20480;					//160*128
	
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
		tft_write_data(_bg_color);
	}		
}

void tft_set_bg_color(u16 bg_color)
{
	_bg_color = bg_color ;
}

void tft_set_text_color( u16 text_color ){
	_text_color = text_color;
}

void tft_set_special_color( u16 special_color ){
	_special_color = special_color;
}


void tft_set_all_color(void){
u8 x , y;	
	
	for(y=0;y<10;y++)
		for(x=0;x<16;x++){
			text_color[y][x] = (text_bg_color_prev[y][x] & 0x40 ? RED : 0) & (text_bg_color_prev[y][x] & 0x20 ? GREEN : 0) & (text_bg_color_prev[y][x] & 0x10 ? BLUE : 0); //text:red green blue
			//text_color[x][y] = (text_bg_color_prev[x][y] & 0x40 ? 0x8000 : 0) & (text_bg_color_prev[x][y] & 0x20 ? 0x0400 : 0) & (text_bg_color_prev[x][y] & 0x10 ? 0x0010 : 0); //text:red green blue
			//bg_color[x][y] = (text_bg_color_prev[x][y] & 0x4 ? 0x8000 : 0) & (text_bg_color_prev[x][y] & 0x2 ? 0x0400 : 0) & (text_bg_color_prev[x][y] & 0x1 ? 0x0010 : 0); //text:red green blue
		}
}

// Horizontal: 0-15
// Vertical: 0-9
void tft_prints(u8 x, u8 y, const u8 * pstr, ...)
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
			if(x >= MAX_WORD)	{
				x = 0;
				y++;
			}
			else{
				x++;
			}
			if( y >= MAX_LINE )
				y = 0;
		}
	}
	is_special = 0;
}


void tft_update(void){
	u8 x , y;
	if (!tft_enabled)
		return;
	for( y = 0 ; y < MAX_LINE ; y ++ ){
		for( x = 0 ; x < MAX_WORD ; x ++ ){
			if ( pre_text[y][x] != text[y][x] || pre_text_color[y][x] != text_color[y][x] ) {
				pre_text[y][x] = text[y][x];
				pre_text_color[y][x] = text_color[y][x];
				tft_put_char(x, y, text[y][x], text_color[y][x] );
			}
		}
	}
}

void tft_force_clear(void) {
	u8 x,y;
	for (y = 0; y < MAX_LINE; y++) {
		for (x = 0; x < MAX_WORD; x++) {
			text[y][x] = ' ';
			text_color[y][x]=0;
		}
	}
	tft_fill_color(_bg_color);
}

void tft_init(u8 orientation, u16 bg_color, u16 normal_color, u16 special_color)
{
	u8 x , y;
	tft_spi_init();
	tft_reset();
	tft_config();
    tft_write_command(0x2C);
	tft_set_bg_color(bg_color);
	tft_set_text_color(normal_color);
	tft_set_special_color(special_color);
	tft_fill_color(_bg_color);
	tft_orientation = orientation;

	for( y = 0 ; y < MAX_LINE ; y ++ ){
		for( x = 0 ; x < MAX_WORD ; x ++ ){
			text[y][x] = ' ';
			text_color[y][x]=0;
		}
	}
}

void tft_clear_line(u8 line ){
	u8 x;
	for( x = 0 ; x < MAX_WORD ; x ++ ){
		text[line][x] = ' ';
		text_color[line][x]=0;
	}
}

void tft_clear(void ){
	u8 y;
	for( y=0; y < MAX_LINE; y++)
		tft_clear_line(y);
}

void tft_toggle(void ){
	tft_force_clear();
	tft_orientation=tft_orientation?0:2;
//	tft_orientation = (tft_orientation+1)%4;
}

void tft_printx(const uc8 * pstr, ...)
{
	u8 buf[256], i, j, is_special = 0;
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
		} else {
			if (*fp == '\r' || *fp == '\n'){
				fp++;
				for (; print_pos%16 != 0; print_pos++) {
					i = print_pos%16;
					j = print_pos/16;
					if (j > 7) {
						print_pos = 128;
						break;
					}
					text[j][i] = ' ';
				}
			}
			else {
				//--- scroll
				if (print_pos >= 128) {
					
					for (i = 0; i < 16; i++) {
						for (j = 0; j < 7; j++) {
							text[j][i] = text[j+1][i];
							text_color[j][i] = text_color[j+1][i];
						}
						text[7][i] = ' ';
					}
					print_pos = 112;
				}

				i = print_pos%16;
				j = print_pos/16;
				text[j][i] = *fp++;
				text_color[j][i] = is_special ? _special_color : _text_color;
				print_pos++;
			}
		}
	}
	is_special = 0;
}

void tft_enable(void) {
	tft_enabled = 1;
}

void tft_disable(void) {
	tft_enabled = 0;
}


void tft_set_pixel_pos(u8 x, u8 y){
	tft_write_command(0x2a);		// Column addr set
	tft_write_data(0x00);
	tft_write_data(x); 				// X START
	tft_write_data(0x00);
	tft_write_data(x+1); 			// X END

	tft_write_command(0x2b);		// Row addr set
	tft_write_data(0x00);
	tft_write_data(y);				// Y START
	tft_write_data(0x00);
	tft_write_data(y+1);			// Y END

	tft_write_command(0x2c); 		// write to RAM
}

void tft_put_pixel(u8 x, u8 y, u16 color)   
{   
	tft_set_pixel_pos(x,y);
    tft_write_data(color>>8);
    tft_write_data(color);
}  




