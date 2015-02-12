#ifndef	__GAME_COUNTER_H
#define	__GAME_COUNTER_H

#include <stdbool.h>
#include "gpio.h"

#define LED_SEGMENT_COUNT   7
#define LED_DIGIT_COUNT     3

#define NO_DIGIT  10


typedef struct {
  const GPIO* gpio;
  bool status; 
} LED_TUBE;


//static LCD_COUNTER game_clock={

//	 0,  GPIO_Pin_10,	GPIOC ,
//	 0,  GPIO_Pin_10,	GPIOC ,

//	 0,  GPIO_Pin_0,	GPIOB ,
//	 0,  GPIO_Pin_4,	GPIOA ,
//	 0,  GPIO_Pin_5,	GPIOA ,
//	 0,  GPIO_Pin_6,	GPIOA ,
//	 0,  GPIO_Pin_7,	GPIOA ,
//	 0,  GPIO_Pin_4,	GPIOC ,
//	 0,  GPIO_Pin_5,	GPIOC ,

//	 0,  GPIO_Pin_1,	GPIOB ,

//	 0,  GPIO_Pin_8,	GPIOC ,
//	 0,  GPIO_Pin_12,	GPIOB ,
//	 0,  GPIO_Pin_13,	GPIOB ,
//	 0,  GPIO_Pin_14,	GPIOB ,
//	 0,  GPIO_Pin_15,	GPIOB ,
//	 0,  GPIO_Pin_7,	GPIOC ,
//	 0,  GPIO_Pin_6,	GPIOC ,

//	 0,  GPIO_Pin_9,	GPIOB ,
//	 0,  GPIO_Pin_11,	GPIOC ,
//	 0,  GPIO_Pin_12,	GPIOC ,
//	 0,  GPIO_Pin_5,	GPIOB ,
//	 0,  GPIO_Pin_6,	GPIOB ,
//	 0,  GPIO_Pin_8,	GPIOB ,
//	 0,  GPIO_Pin_7,	GPIOB ,

//};
void game_counter_init(void);
void game_counter_all_off(void);
void game_counter_tube_set(u8 digit_id, u8 segment, u8 mode);
void game_counter_colon_set(u8 mode);
void game_counter_set_digit_id(u8 digit_id, u8 digit);
void game_counter_set_time(u8 minute, u8 seconds);
void game_counter_display_up(void);

#endif /* __BUZZER_H */
