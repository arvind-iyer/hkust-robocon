/**
  ******************************************************************************
  * @file    button.c
  * @author  Kenneth Au
  * @version V1.0.0
  * @date    01-February-2015
  * @brief   This file provides button functions, including initialization, 
  *          function of button pressed time, button held time, button
  *          released time and update function. 
	*
  ******************************************************************************
  * @attention
  *
  * This source is designed for application use. Unless necessary, try NOT to
	* modify the function definition. The constants which are more likely to 
	* vary among different schematics have been placed as pre-defined constant
	* (i.e., "#define") in the header file.
	*
  ******************************************************************************
  */
  
#include "button.h"

static const GPIO* buttons[BUTTON_COUNT] = {
	BUTTON_1_GPIO,
	BUTTON_2_GPIO,
  BUTTON_4_GPIO
};

static u16 button_pressed_count[BUTTON_COUNT];
static u16 button_released_count[BUTTON_COUNT];


/**
  * @brief  Initialization of button (joystick)'s gpio.
  * @param  None
  * @retval None
  */
void button_init(void)
{
	// GPIO init
	for (u8 i = 0; i < BUTTON_COUNT; ++i) {
		gpio_init(buttons[i], GPIO_Speed_2MHz, GPIO_Mode_IPD, 1);
		button_pressed_count[i] = 0;
		button_released_count[i] = 0;
	}
}

/**
	* @brief Update the button state (pressed count and released state). To be called regularly.
  * @param None
  * @retval None
	*/
void button_update(void)
{
	for (u8 i = 0; i < BUTTON_COUNT; ++i) {
		const GPIO* button = buttons[i];
		if (gpio_read_input(button) == BUTTON_PRESSED) {
			++button_pressed_count[i];
			button_released_count[i] = 0;
		} else {
			if (button_pressed_count[i] != 0) {
				button_released_count[i] = 1;
			} else if (button_released_count[i] > 0) {
				++button_released_count[i];
				if (button_released_count[i] > BUTTON_RELEASED_LIMIT) {
					button_released_count[i] = 0;
				}
			}
			button_pressed_count[i] = 0;
		}
	}
}


/**
  * @brief Get the time (unit depends on button_update) of a button being pressed
  * @param b: The button
  * @retval The time of a button being pressed
  */
u16 button_pressed(BUTTON b)
{
	if (b < BUTTON_COUNT) {
		return button_pressed_count[b];
	} else {
		return 0;
	}
}

/** 
  * @brief Get the Boolean of a button being pressed 
  * @param b: The button
  * @param threshold: A threshold which the function will always return false before it
  * @param mod: The multiple which the function will return true after the threshold
  * @retval Return true when button_pressed(b) returns (threshold + n * mod), where n is a non-negative integer 
  * @example button_hold(b, 18, 7) will return true when button b is pressed with time 18, 25, 32, 39, ...
  */
u8 button_hold(BUTTON b, u16 threshold, u8 mod) {
	return (button_pressed(b) > threshold) && ((button_pressed(b) - threshold) % mod == 0);
}

/**
  * @brief Get the time (depends on button_update) of the button being released
  * @param b: The button
  * @retval The time of the button being released (from 0 to BUTTON_COUNT). 0 when the button is pressed
  */
u16 button_released(BUTTON b) 
{
	if (b < BUTTON_COUNT) {
		return button_released_count[b];
	} else {
		return 0;
	}
}

