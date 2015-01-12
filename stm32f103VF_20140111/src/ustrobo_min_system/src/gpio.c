#include "gpio.h"

const GPIO 	
			/*** GPIOA ***/
			PA0 = {GPIOA, GPIO_Pin_0},
			PA1 = {GPIOA, GPIO_Pin_1},
			PA2 = {GPIOA, GPIO_Pin_2},
			PA3 = {GPIOA, GPIO_Pin_3},
			PA4 = {GPIOA, GPIO_Pin_4},
			PA5 = {GPIOA, GPIO_Pin_5},
			PA6 = {GPIOA, GPIO_Pin_6},
			PA7 = {GPIOA, GPIO_Pin_7},
			PA8 = {GPIOA, GPIO_Pin_8},
			PA9 = {GPIOA, GPIO_Pin_9},
			PA10 = {GPIOA, GPIO_Pin_10},
			PA11 = {GPIOA, GPIO_Pin_11},
			PA12 = {GPIOA, GPIO_Pin_12},

			/*** GPIOB ***/
			PB0 = {GPIOB, GPIO_Pin_0},
			PB1 = {GPIOB, GPIO_Pin_1},
			PB2 = {GPIOB, GPIO_Pin_2},
			PB3 = {GPIOB, GPIO_Pin_3},
			PB4 = {GPIOB, GPIO_Pin_4},
			PB5 = {GPIOB, GPIO_Pin_5},
			PB6 = {GPIOB, GPIO_Pin_6},
			PB7 = {GPIOB, GPIO_Pin_7},
			PB8 = {GPIOB, GPIO_Pin_8},
			PB9 = {GPIOB, GPIO_Pin_9},
			PB10 = {GPIOB, GPIO_Pin_10},
			PB11 = {GPIOB, GPIO_Pin_11},
			PB12 = {GPIOB, GPIO_Pin_12},
			PB13 = {GPIOB, GPIO_Pin_13},
			PB14 = {GPIOB, GPIO_Pin_14},
			PB15 = {GPIOB, GPIO_Pin_15},

			/*** GPIOC ***/
			PC0 = {GPIOC, GPIO_Pin_0},
			PC1 = {GPIOC, GPIO_Pin_1},
			PC2 = {GPIOC, GPIO_Pin_2},
			PC3 = {GPIOC, GPIO_Pin_3},
			PC4 = {GPIOC, GPIO_Pin_4},
			PC5 = {GPIOC, GPIO_Pin_5},
			PC6 = {GPIOC, GPIO_Pin_6},
			PC7 = {GPIOC, GPIO_Pin_7},
			PC8 = {GPIOC, GPIO_Pin_8},
			PC9 = {GPIOC, GPIO_Pin_9},
			PC10 = {GPIOC, GPIO_Pin_10},
			PC11 = {GPIOC, GPIO_Pin_11},
			PC12 = {GPIOC, GPIO_Pin_12},
			PC13 = {GPIOC, GPIO_Pin_13},
			PC14 = {GPIOC, GPIO_Pin_14},
			PC15 = {GPIOC, GPIO_Pin_15},
			
			/*** GPIOD ***/
			PD0 = {GPIOD, GPIO_Pin_0},
			PD1 = {GPIOD, GPIO_Pin_1},
			PD2 = {GPIOD, GPIO_Pin_2},
			PD3 = {GPIOD, GPIO_Pin_3},
			PD4 = {GPIOD, GPIO_Pin_4},
			PD5 = {GPIOD, GPIO_Pin_5},
			PD6 = {GPIOD, GPIO_Pin_6},
			PD7 = {GPIOD, GPIO_Pin_7},
			PD8 = {GPIOD, GPIO_Pin_8},
			PD9 = {GPIOD, GPIO_Pin_9},
			PD10 = {GPIOD, GPIO_Pin_10},
			PD11 = {GPIOD, GPIO_Pin_11},
			PD12 = {GPIOD, GPIO_Pin_12},
			PD13 = {GPIOD, GPIO_Pin_13},
			PD14 = {GPIOD, GPIO_Pin_14},
			PD15 = {GPIOD, GPIO_Pin_15},

			/*** GPIOE ***/
			PE0 = {GPIOE, GPIO_Pin_0},
			PE1 = {GPIOE, GPIO_Pin_1},
			PE2 = {GPIOE, GPIO_Pin_2},
			PE3 = {GPIOE, GPIO_Pin_3},
			PE4 = {GPIOE, GPIO_Pin_4},
			PE5 = {GPIOE, GPIO_Pin_5},
			PE6 = {GPIOE, GPIO_Pin_6},
			PE7 = {GPIOE, GPIO_Pin_7},
			PE8 = {GPIOE, GPIO_Pin_8},
			PE9 = {GPIOE, GPIO_Pin_9},
			PE10 = {GPIOE, GPIO_Pin_10},
			PE11 = {GPIOE, GPIO_Pin_11},
			PE12 = {GPIOE, GPIO_Pin_12},
			PE13 = {GPIOE, GPIO_Pin_13},
			PE14 = {GPIOE, GPIO_Pin_14},
			PE15 = {GPIOE, GPIO_Pin_15},

			/*** GPIOF ***/
			PF0 = {GPIOF, GPIO_Pin_0},
			PF1 = {GPIOF, GPIO_Pin_1},
			PF2 = {GPIOF, GPIO_Pin_2},
			PF3 = {GPIOF, GPIO_Pin_3},
			PF4 = {GPIOF, GPIO_Pin_4},
			PF5 = {GPIOF, GPIO_Pin_5},
			PF6 = {GPIOF, GPIO_Pin_6},
			PF7 = {GPIOF, GPIO_Pin_7},
			PF8 = {GPIOF, GPIO_Pin_8},
			PF9 = {GPIOF, GPIO_Pin_9},
			PF10 = {GPIOF, GPIO_Pin_10},
			PF11 = {GPIOF, GPIO_Pin_11},
			PF12 = {GPIOF, GPIO_Pin_12},
			PF13 = {GPIOF, GPIO_Pin_13},
			PF14 = {GPIOF, GPIO_Pin_14},
			PF15 = {GPIOF, GPIO_Pin_15},

			/*** GPIOG ***/
			PG0 = {GPIOG, GPIO_Pin_0},
			PG1 = {GPIOG, GPIO_Pin_1},
			PG2 = {GPIOG, GPIO_Pin_2},
			PG3 = {GPIOG, GPIO_Pin_3},
			PG4 = {GPIOG, GPIO_Pin_4},
			PG5 = {GPIOG, GPIO_Pin_5},
			PG6 = {GPIOG, GPIO_Pin_6},
			PG7 = {GPIOG, GPIO_Pin_7},
			PG8 = {GPIOG, GPIO_Pin_8},
			PG9 = {GPIOG, GPIO_Pin_9},
			PG10 = {GPIOG, GPIO_Pin_10},
			PG11 = {GPIOG, GPIO_Pin_11},
			PG12 = {GPIOG, GPIO_Pin_12},
			PG13 = {GPIOG, GPIO_Pin_13},
			PG14 = {GPIOG, GPIO_Pin_14},
			PG15 = {GPIOG, GPIO_Pin_15}
			
			;

void gpio_init(GPIO gpio, GPIOSpeed_TypeDef speed, GPIOMode_TypeDef mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = speed;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_Pin = gpio.gpio_pin;
	GPIO_Init(gpio.gpio, &GPIO_InitStructure);
}

void gpio_rcc_init(GPIO gpio)
{
	switch ((u32) gpio.gpio) {
		case ((u32)GPIOA):
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		break;
		
		case ((u32)GPIOB):
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		break;
		
		case ((u32)GPIOC):
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		break;
		
		case ((u32)GPIOD):
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
		break;
		
		case ((u32)GPIOE):
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
		break;
		
		case ((u32)GPIOF):
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
		break;
		
		case ((u32)GPIOG):
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
		break;
	}
}


