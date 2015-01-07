
/****************************** ST7735S TFT SCREEN LIBRARY ************************************
 * Last modified: 13/11/2014
 * Last modified by: GAO Zihou
 * This is a new version of TFT library, which handle the screen update in ticks
 * The motivation is that update screen will take a lot of time normally, which can greatly
 * affect the running of the main program. So we use this new library to handle the TFT screen
***********************************************************************************************/

#ifndef _TFT_H_
#define _TFT_H_

#include "stm32f10x_gpio.h"
#include "stm32f10x.h"
#include  "stm32f10x_spi.h"
#include "delay.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdarg.h>

/** @attention
  * These are configuration data for init of SPI, RST, DC and CS
  * You must modify all of them whenever you have a new PCB board here
**/
#define TFT_SPI			  SPI1
#define TFT_SPI_RCC   RCC_APB2Periph_SPI1

#define TFT_RST_PORT	GPIOD
#define TFT_DC_PORT		GPIOD
#define GPIO_CS			  GPIOA
#define TFT_CLK_PORT	GPIOB
#define TFT_MOSI_PORT	GPIOB

#define TFT_RST_PIN		GPIO_Pin_7
#define TFT_DC_PIN		GPIO_Pin_6
#define GPIO_Pin_CS		GPIO_Pin_15
#define TFT_CLK_PIN	  GPIO_Pin_3
#define TFT_MOSI_PIN	GPIO_Pin_5

#define RCC_APB2Periph_GPIO_RST	   RCC_APB2Periph_GPIOD
#define RCC_APB2Periph_GPIO_DC	   RCC_APB2Periph_GPIOD
#define RCC_APB2Periph_GPIO_CS	   RCC_APB2Periph_GPIOA
#define RCC_APB2Periph_GPIO_CLK	   RCC_APB2Periph_GPIOB
#define RCC_APB2Periph_GPIO_MOSI   RCC_APB2Periph_GPIOB


/** @explanation
  * Max number of chars that the queue could handle.
	* The recommended number is 44100 / FREQ_OF_TICKS
	* The RAM needed is around (QUEUE_SIZE * 7 + 2000)bits
**/
#define QUEUE_SIZE    1024 

/** @explanation
  * How many times the "lcd_update_char()" function loops one time. The larger, the faster
	* The recommended number is 44100 / FREQ_OF_TICKS
	* FREQ_OF_TICKS is the frequency that you set for the timer handler
**/
#define UPDATE_SPEED_LEVEL    2 

/** @attention
  * We should define the following TFT color sequence, because the color sequence
  * vary from screen to screen. We should test it by setting the TFT screen to be red
  * and see if the color is red. If the color is red, then the sequence is RGB, otherwise
  * the color is BGR. 
**/
#if !defined TFT_SCREEN_COLOR_RGB && !defined TFT_SCREEN_COLOR_BGR
	/* #define TFT_SCREEN_COLOR_RGB */  //The sequence is red, green, blue
	   #define TFT_SCREEN_COLOR_BGR     //The sequence is blue, green, red
#endif

/** @attention
  * The following functions are recommended to use to print
  * Note that calling these functions does not mean they can be printed immediately
	* Just add the char data into a buffer
**/
/* TFT screen initialization */
void tft_init(u8 orientation, u16 bg_color, u16 normal_color, u16 special_color);

/* Set background color */
void tft_set_bg_color(u16 bg_color);

/* Set text color */
void tft_set_text_color(u16 text_color);

/* Set special color */
void tft_set_special_color( u16 special_color );

/* Print out the string */
void tft_string_prints(u8 x, u8 y, const char* a);

/* Print out the u16 integer */
void tft_u16_prints(u8 x, u8 y, u16 a);

/* Print out the u8 integer */
void tft_u8_prints(u8 x, u8 y, u8 a);

/* Clear one line */
void tft_clear_line(u8 line );

/* Clear whole screen */
void tft_clear(void);	

/** @attention
  * The following functions are recommended to use to update the screen
  * It is recommended to call the first function at the end of your control cycle, for example, 200Hz
	* Later then you should call the second function in the Ticks_Handler of the same timer as your main control
	* Which means you call the second function every ticks
**/
/* must be called, basically add all the changed items into the queue */
void tft_update(void);			

/* should be implemented in a highly_scheduled ticks handler, put characters in the queue onto the screen */
void lcd_update_char(void);

/* Color in BGR */
#ifdef TFT_SCREEN_COLOR_BGR
#define WHITE          0xFFFF
#define BLACK          0x0000
#define GREY           0x8410
#define BLUE           0xF800
#define RED            0x001F
#define MAGENTA        0xFB1F
#define GREEN          0x07E0
#define CYAN           0xDF8D
#define YELLOW         0x07FF
#define STRANGE_BLUE   0xFE4C
#define PURPLE         0xF1D7
#define ORANGE         0x2D5F
#define BROWN          0x2B9A
#define DARK_GREEN     0x552F
#define ACID_BLUE      0xFFC0
#define GRASS_GREEN    0x07C8
#define STRANGE_GREEN  0x2E88
#define DARK_PURPLE    0x8251
#define CHINA_RED      0x033F
#define PINK           0x7AFF
#endif /* TFT_SCREEN_COLOR_BGR */

/* Color in RGB */
#ifdef TFT_SCREEN_COLOR_RGB
#define WHITE          0xFFF
#define BLACK          0x000
#define GREY           0x777
#define BLUE           0x00F
#define RED            0xF00
#define MAGENTA        0xF1F
#define GREEN          0x0F0
#define CYAN           0x0FF
#define YELLOW         0xFF0
#define STRANGE_BLUE   0x3EC
#define PURPLE         0xE4D
#define ORANGE         0xFA0
#define BROWN          0x740
#define DARK_GREEN     0x2C1
#define ACID_BLUE      0x3CE
#define GRASS_GREEN    0x3E3
#define STRANGE_GREEN  0x3E8
#define DARK_PURPLE    0xD49
#define CHINA_RED      0xF31
#endif /* TFT_SCREEN_COLOR_RGB */

/* Fixed parameters for the TFT */
#define HORIZONTAL_PIXEL_NO				128
#define VERTICAL_PIXEL_NO				160
#define WORD_HORIZONTAL_PIXEL_NO		8
#define WORD_VERTICAL_PIXEL_NO			16
#define MAX_WORD						HORIZONTAL_PIXEL_NO/WORD_HORIZONTAL_PIXEL_NO
#define MAX_LINE						VERTICAL_PIXEL_NO/WORD_VERTICAL_PIXEL_NO

/** @attention
  * The following functions are not recommended to use. 
	* If you must use it, take care and good luck...
**/
/* General print function. Too slow, not recommended */
void tft_prints(u8 x, u8 y, const char * pstr, ...);
/* clear the screen at once. May slow down the MCU a lot and affect normal tasks */
void tft_force_clear(void);		
/* the base code to directly add one char into the queue. Risk of queue overflow */
void lcd_print_queue_enqueue(u8 a, u8 b, u8 c, u16 color, u16 bg_color);  

void print_picture(u8* a);
void tft_master_clear(void);
void tft_fill_color(u16 color);
u16 get_curr_bg_color(void);
void print_picture_from_SD(u32 address);

#endif		/* _TFT_H_ */
