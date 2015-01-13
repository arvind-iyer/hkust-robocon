#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include "stm32f10x_tim.h"

#define BUTTON_DIR_GPIO				GPIOA													// Direction key's GPIO
#define BUTTON_DIR_GPIO_RCC		RCC_APB2Periph_GPIOA					// Direction key's GPIO_RCC
#define BUTTON_CLICK_GPIO 		GPIOC													// Clicking joystick's GPIO
#define BUTTON_CLICK_GPIO_RCC RCC_APB2Periph_GPIOC					// Clicking joystick's GPIO_RCC
#define BUTTON_UP 						GPIO_Pin_4
#define BUTTON_LEFT 					GPIO_Pin_5
#define BUTTON_DOWN 					GPIO_Pin_6
#define BUTTON_RIGHT 					GPIO_Pin_7
#define BUTTON_CENTER					GPIO_Pin_4


static const GPIO* BUTTON_UP_GPIO = &PA4;
static const GPIO* BUTTON_LEFT_GPIO = &PA5;
static const GPIO* BUTTON_DOWN_GPIO = &PA6;
static const GPIO* BUTTON_RIGHT_GPIO = &PA7;
static const GPIO* BUTTON_CENTER_GPIO = &PC4;

void button_init(void);

#endif /* __BUTTON_H */

