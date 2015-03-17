#include "button.h"

u8 button_prev_data=255;
u8 button_curr_data=255;
u8 button_down=255;
u8 button_data=255;
u8 _psc_flag=0;

void button_init(void){

	GPIO_InitTypeDef GPIO_InitStructure;

  	RCC_APB2PeriphClockCmd(BUT_RCC|RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  	GPIO_InitStructure.GPIO_Pin = BUT_START|BUT_UP|BUT_DOWN|BUT_L|BUT_R;
  	GPIO_Init(BUT_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = BUT_WAKEUP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void button_update(void){
	button_prev_data = button_curr_data;
	if (tft_orientation == 0) {
		button_curr_data = (GPIO_ReadInputDataBit(BUT_PORT,BUT_UP) << 1);
		button_curr_data |= (GPIO_ReadInputDataBit(BUT_PORT,BUT_DOWN));
		button_curr_data |= (GPIO_ReadInputDataBit(BUT_PORT,BUT_L) << 3);
		button_curr_data |= (GPIO_ReadInputDataBit(BUT_PORT,BUT_R) << 2);
	} else if (tft_orientation == 1) {
		button_curr_data = (GPIO_ReadInputDataBit(BUT_PORT,BUT_UP) << 2);
		button_curr_data |= (GPIO_ReadInputDataBit(BUT_PORT,BUT_DOWN) << 3);
		button_curr_data |= (GPIO_ReadInputDataBit(BUT_PORT,BUT_L) << 1);
		button_curr_data |= (GPIO_ReadInputDataBit(BUT_PORT,BUT_R));
	} else if (tft_orientation == 2) {
		button_curr_data = GPIO_ReadInputDataBit(BUT_PORT,BUT_UP);
		button_curr_data |= (GPIO_ReadInputDataBit(BUT_PORT,BUT_DOWN) << 1);
		button_curr_data |= (GPIO_ReadInputDataBit(BUT_PORT,BUT_L) << 2);
		button_curr_data |= (GPIO_ReadInputDataBit(BUT_PORT,BUT_R) << 3);
	} else {		// =3
		button_curr_data = (GPIO_ReadInputDataBit(BUT_PORT,BUT_UP) << 3);
		button_curr_data |= (GPIO_ReadInputDataBit(BUT_PORT,BUT_DOWN) << 2);
		button_curr_data |= (GPIO_ReadInputDataBit(BUT_PORT,BUT_L));
		button_curr_data |= (GPIO_ReadInputDataBit(BUT_PORT,BUT_R) << 1);
	}

	button_curr_data |= ((GPIO_ReadInputDataBit(BUT_PORT,BUT_START)) << 4);
	button_curr_data |= ((GPIO_ReadInputDataBit(GPIOA,BUT_WAKEUP)) << 5);
	

	button_curr_data = ~button_curr_data;
	button_data=button_curr_data;

	button_down = button_curr_data & (button_curr_data ^ button_prev_data);	 
	 
	if (psc_mode) {
		if(~psc_data[PSC_BUT1] & PSC_UP){
			if(!_psc_flag){
				button_down |= _BV(0);
				_psc_flag=1;
			}
		}
		else if(~psc_data[PSC_BUT1] & PSC_DOWN){
			if(!_psc_flag){
				button_down |= _BV(1);	
				_psc_flag=1;
			}
		}
		else if(~psc_data[PSC_BUT1] & PSC_LEFT){
			if(!_psc_flag){
				button_down |= _BV(2);
				_psc_flag=1;
			}
			button_data |= _BV(2);
		}
		else if(~psc_data[PSC_BUT1] & PSC_RIGHT){
			if(!_psc_flag){
				button_down |= _BV(3);
				_psc_flag=1;
			}
			button_data |= _BV(3);
		}
		else if(~psc_data[PSC_BUT1] & PSC_START){
			if(!_psc_flag){
				button_down |= _BV(4);
				_psc_flag=1;
			}
			button_data |= _BV(4);
		}
		else if(~psc_data[PSC_BUT1] & PSC_SELECT){
			if(!_psc_flag){
				button_down |= _BV(5);
				_psc_flag=1;
			}
		}
		else 
			_psc_flag=0;
	}	 	
}





