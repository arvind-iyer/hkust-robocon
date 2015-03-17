#ifndef _TFT_H_
#define _TFT_H_

#include "stm32f10x_gpio.h"
#include "delay.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdarg.h>

/*
 *  **for Xbox controller only**
 */
 
// SPI, RST, DC
#define TFT_RST_PIN		GPIO_Pin_9
#define TFT_DC_PIN		GPIO_Pin_11
#define TFT_RST_PORT	GPIOB
#define TFT_DC_PORT		GPIOB

#define TFT_SPI			SPI1
#define TFT_SPI1_PORT   RCC_APB2Periph_GPIOA
#define GPIO_Pin_CS		GPIO_Pin_4
#define GPIO_Pin_SCK	GPIO_Pin_5
#define GPIO_Pin_MISO	GPIO_Pin_6
#define GPIO_Pin_MOSI	GPIO_Pin_7

#define RCC_APB2Periph_GPIO_CS	RCC_APB2Periph_GPIOA
#define GPIO_CS			GPIOA


// Color
#define WHITE          0xFFFF
#define BLACK          0x0000
#define GREY           0xF7DE
#define BLUE           0x001F
#define BLUE2          0x051F
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0



#define HORIZONTAL_PIXEL_NO				128
#define VERTICAL_PIXEL_NO				160
#define WORD_HORIZONTAL_PIXEL_NO		8
#define WORD_VERTICAL_PIXEL_NO			16
#define MAX_WORD						HORIZONTAL_PIXEL_NO/WORD_HORIZONTAL_PIXEL_NO
#define MAX_LINE						VERTICAL_PIXEL_NO/WORD_VERTICAL_PIXEL_NO

extern u8 tft_orientation;
extern u8 text_bg_color_prev[MAX_LINE][MAX_WORD];

void tft_init(u8 orientation, u16 bg_color, u16 normal_color, u16 special_color);		//init
void tft_prints(u8 x, u8 y, const u8 * pstr, ...);		//x-coordinate, y-coordinate, string
void tft_update(void);			//update screen, needed to be called for updating the screen otherwise no changes would be made
void tft_clear_line(u8 line );	//clear one line 
void tft_clear(void);			//clear whole screen	
void tft_toggle(void );			//toggle orientationvoid tft_set_pixel_pos(u8 x, u8 y)
void tft_force_clear(void);		//clear the screen at once
void tft_put_pixel(u8 x, u8 y, u16 color); //output one pixel
void tft_set_text_color(u16 text_color);			//set text color
void tft_set_all_color(void);

#endif		/* _TFT_H_ */

