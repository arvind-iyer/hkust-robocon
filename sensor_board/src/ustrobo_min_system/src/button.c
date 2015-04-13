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
#ifdef MAINBOARD_V4
	BUTTON_JS_UP_GPIO,
	BUTTON_JS_LEFT_GPIO,
	BUTTON_JS_DOWN_GPIO,
	BUTTON_JS_RIGHT_GPIO,
	BUTTON_JS_CENTER_GPIO,
#else
	BUTTON_JS1_UP_GPIO,
	BUTTON_JS1_LEFT_GPIO,
	BUTTON_JS1_DOWN_GPIO,
	BUTTON_JS1_RIGHT_GPIO,
	BUTTON_JS1_CENTER_GPIO,
	BUTTON_JS2_UP_GPIO,
	BUTTON_JS2_LEFT_GPIO,
	BUTTON_JS2_DOWN_GPIO,
	BUTTON_JS2_RIGHT_GPIO,
	BUTTON_JS2_CENTER_GPIO,
#endif
	BUTTON_1_GPIO,
	BUTTON_2_GPIO
};

static u16 button_pressed_count[BUTTON_COUNT + XBC_BUTTON_COUNTS];
static u16 button_released_count[BUTTON_COUNT + XBC_BUTTON_COUNTS];


/**
  * @brief  Initialization of button (joystick)'s gpio.
  * @param  None
  * @retval None
  */
void button_init(void)
{
	// GPIO init
	for (u8 i = 0; i < BUTTON_COUNT; ++i) {
		gpio_init(buttons[i], GPIO_Speed_2MHz, GPIO_Mode_IPU, 1);
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
  u32 xbox_tmp = 1;
	for (u8 i = 0; i < BUTTON_COUNT + XBC_BUTTON_COUNTS; ++i) {
    u8 button_pressed_flag = 0;
    
    if (i < XBC_BUTTON_START_ID) {
      const GPIO* button = buttons[i];
      button_pressed_flag = (gpio_read_input(button) == BUTTON_PRESSED);
    } else {
      // XBOX Buttons
      // Special case (Diagonal switch)
      switch (i) {
          /** Special cases **/
          case BUTTON_XBC_NE:
            button_pressed_flag = (xbc_get_digital() & XBC_UP) && (xbc_get_digital() & XBC_RIGHT);
          break;
          case BUTTON_XBC_SE:
            button_pressed_flag = (xbc_get_digital() & XBC_DOWN) && (xbc_get_digital() & XBC_RIGHT);
          break;
          case BUTTON_XBC_SW:
            button_pressed_flag = (xbc_get_digital() & XBC_DOWN) && (xbc_get_digital() & XBC_LEFT);
          break;
          case BUTTON_XBC_NW:
            button_pressed_flag = (xbc_get_digital() & XBC_UP) && (xbc_get_digital() & XBC_LEFT);
          break;
            
          
          default: 
            // Normal case
            button_pressed_flag = (xbc_get_digital() & xbox_tmp) > 0;
            if (i == BUTTON_XBC_N || i == BUTTON_XBC_E || i == BUTTON_XBC_S || i == BUTTON_XBC_W) {
              // Diagonal press will be treated as 0
              u8 pressed_count = ((xbc_get_digital() & XBC_UP) > 0) + ((xbc_get_digital() & XBC_RIGHT) > 0)
                + ((xbc_get_digital() & XBC_DOWN) > 0) + ((xbc_get_digital() & XBC_LEFT) > 0);
              if (pressed_count != 1) {
                button_pressed_flag = 0;
              }
            }
            
            xbox_tmp <<= 1;
          break;
      }
      
    }
    
		if (button_pressed_flag) {
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
  * @brief Private function of rotating the joystick based on the current LCD orientation
  * @param b: the button
  * @retval The button after rotation (non-joystick button will remain the same)
  */
static BUTTON rotate_js_button(BUTTON b) {
	u8 o = tft_get_orientation();
	
	
	#ifdef MAINBOARD_V4
	if (b <= 3) {
		b = (BUTTON) ((b + 4 - o + 1) % 4);
	}
	#else
	if (b <= 3) {
		b = (BUTTON) ((b + 4 - o + 1) % 4);
	}
	
	if (b >= 5 && b <= 8) {
		b -= 5;
		b = (BUTTON) ((b + 4 - o + 1) % 4);
		b += 5;
	}
	#endif
	
	return b;
}

/**
  * @brief Get the time (unit depends on button_update) of a button being pressed
  * @param b: The button
  * @retval The time of a button being pressed
  */
u16 button_pressed(BUTTON b)
{
	if (b < BUTTON_COUNT + XBC_BUTTON_COUNTS) {
		return button_pressed_count[rotate_js_button(b)];
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
	if (b < BUTTON_COUNT + XBC_BUTTON_COUNTS) {
		return button_released_count[rotate_js_button(b)];
	} else {
		return 0;
	}
}

