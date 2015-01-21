#include "button.h"

static const GPIO* buttons[BUTTON_COUNT] = {
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
	BUTTON_1_GPIO,
	BUTTON_2_GPIO
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
		gpio_init(buttons[i], GPIO_Speed_2MHz, GPIO_Mode_IPU, 1);
		button_pressed_count[i] = 0;
		button_released_count[i] = 0;
	}
}

/**
	* @brief Update the button state (pressed count and released state)
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

static BUTTON rotate_js_button(BUTTON b) {
	u8 o = tft_get_orientation();
	if (b >= 0 && b <= 3) {
		b = (b + 4 - o + 1) % 4;
	}
	
	if (b >= 5 && b <= 8) {
		b -= 5;
		b = (b + 4 - o + 1) % 4;
		b += 5;
	}
	
	return b;
}

u16 button_pressed(BUTTON b)
{
	if (b < BUTTON_COUNT) {
		return button_pressed_count[rotate_js_button(b)];
	} else {
		return 0;
	}
}

u8 button_hold(BUTTON b, u16 threshold, u8 mod) {
	return button_pressed(b) > threshold && (button_pressed(b) - threshold) % mod;
}


u16 button_released(BUTTON b) 
{
	if (b < BUTTON_COUNT) {
		return button_released_count[rotate_js_button(b)];
	} else {
		return 0;
	}
}
