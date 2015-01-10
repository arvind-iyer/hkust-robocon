#include "button.h"

u8 button_prev_data = 0;
u8 button_curr_data = 0;
u8 button_down = 0;
u8 button_data = 0;
u8 button_wakeup_prev = 1;
u16 button_wakeup_cnt = 0;

u8 xbc_down_flag = 0;

/**
  * @brief  Inintialization of Buttons
  * @param  None
  * @retval None
  */
void button_init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;

  	RCC_APB2PeriphClockCmd(BUT_RCC|RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	GPIO_InitStructure.GPIO_Pin = BUT_START | BUT_UP | BUT_DOWN | BUT_L | BUT_R;
  	GPIO_Init(BUT_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = BUT_WAKEUP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @brief  Updating the states of Buttons
  * @param  None
  * @retval None
  */
void button_update(void)
{
	button_prev_data = button_curr_data;
	button_curr_data = 0;
	if (tft_orientation == 0) {
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_UP) << 1);
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_DOWN));
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_L) << 3);
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_R) << 2);
	} else if (tft_orientation == 1) {
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_UP) << 3);
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_DOWN) << 2);
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_L));
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_R) << 1);
	} else if (tft_orientation == 2) {
		button_curr_data |= !GPIO_ReadInputDataBit(BUT_PORT,BUT_UP);
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_DOWN) << 1);
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_L) << 2);
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_R) << 3);
	} else {
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_UP) << 2);
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_DOWN) << 3);
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_L) << 1);
		button_curr_data |= (!GPIO_ReadInputDataBit(BUT_PORT,BUT_R));
	}

	button_curr_data |= ((!GPIO_ReadInputDataBit(BUT_PORT, BUT_START)) << 4);
	if (button_wakeup_cnt > 0) {
		button_wakeup_cnt--;
	} else {
		if (button_wakeup_prev != GPIO_ReadInputDataBit(GPIOA, BUT_WAKEUP)) {
			button_wakeup_prev = GPIO_ReadInputDataBit(GPIOA, BUT_WAKEUP);
			button_wakeup_cnt = 50;		// >=50ms for debouncing
		}
	}
	button_curr_data |= (button_wakeup_prev << 5);

	button_data = button_curr_data;
	button_down = button_curr_data & (button_curr_data ^ button_prev_data);
	
	if (get_xbc_mode() == 1) {
		xbc_update();
		if (xbc_digital & XBC_UP || xbc_joy[XBC_LY] > 28000) {
			if (!xbc_down_flag) {
				button_down |= 1;
				xbc_down_flag = 1;
			}
		} else if (xbc_digital & XBC_DOWN || xbc_joy[XBC_LY] < -28000) {
			if (!xbc_down_flag) {
				button_down |= (1 << 1);
				xbc_down_flag = 1;
			}
		} else if (xbc_digital & XBC_LEFT || xbc_joy[XBC_LX] < -28000) {
			if (!xbc_down_flag) {
				button_down |= (1 << 2);
				xbc_down_flag = 1;
			}
			button_data |= (1 << 2);
		} else if (xbc_digital & XBC_RIGHT || xbc_joy[XBC_LX] > 28000) {
			if (!xbc_down_flag) {
				button_down |= (1 << 3);
				xbc_down_flag = 1;
			}
			button_data |= (1 << 3);
		} else if (xbc_digital & XBC_START) {
			if (!xbc_down_flag) {
				button_down |= (1 << 4);
				xbc_down_flag = 1;
			}
			button_data |= (1 << 4);
		} else if (xbc_digital & XBC_BACK) {
			if (!xbc_down_flag) {
				button_down |= (1 << 5);
				xbc_down_flag = 1;
			}
		} else {
			xbc_down_flag = 0;
		}
	}
	
}
