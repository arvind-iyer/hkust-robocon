#include "system.h"

char menu_title[MENU_MAX][32];
void(*menu_func[MENU_MAX])(void);
u8 menu_index = 1;

/**
  * @brief  Starting page after boot, showing the name of the program
  * @param  title[50]: program name to be shown
  * @param  cnt: time to wait (in ms), which can be skip by pressing any keys
  * @retval None
  */
void menu_start(char title[50], u16 cnt)
{
	char tmp[CHAR_MAX_X-2] = "";
	cnt >>= 2;
	tft_clear();
	tft_prints(1, 1, "[HKUST]");
	tft_prints(1, 2, "[Robotics Team]");
	
	strncpy(tmp, title, tft_width-2);
	tft_prints(1, 4, "%s", tmp);
	strncpy(tmp, &title[tft_width-2], tft_width-2);
	tft_prints(1, 5, "%s", tmp);
	
	tft_update();
	
	buzzer_control(2, 5);
	while (1) {
		button_update();
		if (!(cnt--) || button_data) {
			break;
		}
		_delay_ms(4);
	}
	tft_force_clear();
}

/**
  * @brief  Function to build the menu
  * @param  index0: default selected index of the menu (starting from 1)
  * @retval None
  */
void menu(u8 index0)
{
	u8 page = 0, page_max = 0;
	u8 index_max = 0, index_per = 0, index_per_max = 0, pressed = 1, toggled = 1;
	s8 i = 0, j = 0, pos = 0;
	u16 seconds = 0, seconds_img = 0;
	u16 clr1 = 0, clr2 = 0, batt = sample_battery(), batt_w = 0;
	
	index_per_max = tft_height-1;
	page = (index0-1)/index_per_max;
	menu_index = (index0-1) % index_per_max;
	
	tft_clear();
	
	for (i = MENU_MAX-1; i >= 0; i--) {
		if (menu_title[i][0] != '\0') {
			index_max = i+1;
			break;
		}
	}
	
	while (1) {
		button_update();
		if (pressed) {
			index_per_max = tft_height-1;
			page_max = (index_max-1)/index_per_max+1;
			if (menu_index >= index_per_max) {
				page = menu_index/index_per_max;
				menu_index = menu_index % index_per_max;
			}
			index_per = (page == page_max-1) ? (((index_max-1) % index_per_max)+1) : index_per_max;
			
			for (i = index_per_max*page; i < index_per_max*page+index_per; i++) {
				if (i % index_per_max == menu_index) {
					clr1 = curr_text_color;
					tft_set_text_color(curr_bg_color);
					tft_set_bg_color(clr1);
				}
				tft_clear_line((i % index_per_max)+1);
				tft_prints(0, (i % index_per_max)+1, "%c %s", (i < 9 ? 49+i : 56+i), menu_title[i]);
				if (i % index_per_max == menu_index) {
					tft_set_bg_color(curr_text_color);
					tft_set_text_color(clr1);
				}
			}
			pressed = 0;
		}
		
		if (button_down & WAKEUP) {
			tft_toggle();
			menu_index = menu_index+page*index_per_max;
			page = 0;
			toggled = 1;
			pressed = 1;
		} else if (button_down & UP) {
			menu_index = (menu_index+index_per-1) % index_per;
			pressed = 1;
		} else if (button_down & DOWN) {
			menu_index = (menu_index+1) % index_per;
			pressed = 1;
		} else if (page_max > 1 && button_down & LEFT) {
			page = (page+page_max-1) % page_max;
			pressed = 1;
		} else if (page_max > 1 && button_down & RIGHT) {
			page = (page+1) % page_max;
			pressed = 1;
		} else if (button_down & START) {
			if (menu_title[menu_index+page*index_per_max][0] != '\0') {
				menu_func[menu_index+page*index_per_max]();
			}
			tft_clear();
			toggled = 1;
			pressed = 1;
		}
		
		if (page_max > 1 && button_down & (LEFT | RIGHT)) {
			menu_index = (page == page_max-1) && (menu_index >= (index_max % (tft_height-1))) ? (index_max % (tft_height-1))-1 : menu_index;
			for (i = 1; i < index_per_max+1; i++)
				tft_clear_line(i);
		}
		
		tft_update();
		
		seconds = get_seconds();
		if (seconds != seconds_img || toggled) {
			seconds_img = seconds;
			
			if (seconds_img % 5 == 0)
				batt = sample_battery();
			
			clr1 = curr_text_color;
			clr2 = curr_bg_color;
			tft_set_text_color(GREY);
			tft_set_bg_color(DARK_GREY);
			if (toggled)
				tft_clear_line(0);
			tft_prints(1, 0, "%c%c:%c%c", ((seconds/60)/10)+48, ((seconds/60)%10)+48, ((seconds%60)/10)+48, ((seconds%60)%10)+48);
			if (batt >= 60) {
				tft_prints(tft_width-7, 0, "%2d.%d", batt/10, batt%10);
			} else {
				tft_set_text_color(RED);
				tft_prints(tft_width-4, 0, "USB");
			}
			tft_set_text_color(clr1);
			tft_set_bg_color(clr2);
			
			if (toggled)
				tft_update();
			
			if (seconds_img % 5 == 0 || toggled) {
				if (batt >= 60) {		// not USB
					pos = (tft_orientation % 2 ? MAX_HEIGHT : MAX_WIDTH);
					clr1 = batt <= 114 ? RED : (batt <= 120 ? 0xFCC0 : GREEN);
					clr2 = batt <= 114 ? RED : WHITE;
					batt_w = (batt > 126 ? 13 : batt < 110 ? 0 : (batt-110)*13/16);
					for (i = 0; i < 13; i++) {
						for (j = 0; j < 6; j++) {
							tft_put_pixel(pos-19+i, 5+j, i < batt_w ? clr1 : DARK_GREY);
						}
					}
					for (i = 0; i < 17; i++) {
						tft_put_pixel(pos-21+i, 3, clr2);
						tft_put_pixel(pos-21+i, 12, clr2);
					}
					for (i = 0; i < 8; i++) {
						tft_put_pixel(pos-21, 4+i, clr2);
						tft_put_pixel(pos-5, 4+i, clr2);
					}
					for (i = 0; i < 6; i++)
						tft_put_pixel(pos-4, 5+i, clr2);
					
					if (batt < 90)
						battery_low();		// exited
				}
				
				toggled = 0;
			}
		}
		
	}
}

/**
  * @brief  Adding an item to the menu
  * @param  i: index of the item (starting by 1)
  * @param  title[32]: title of the function
  * @param  *func: function to jump
  * @retval None
  */
void menu_add(u8 i, char title[32], void (*func)(void))
{
	i--;
	if (i >= 32)
		return;
	
	strcpy(menu_title[i], title);
	menu_func[i] = func;
}

/**
  * @brief  Blank function
  * @param  None
  * @retval None
  */
void blank(void)
{
}

/**
  * @brief  LM629 test
  * @param  None
  * @retval None
  */
#ifdef __LM629_H
void lm629_test(void)
{
	u8 i, index = 0;
	
	tft_clear();
	for (i = 0; i < 5; i++) {
		tft_prints(0, i, "#%d:", (i<<1));
		if (lm629_chip_available & (1 << (i*2))) {
			tft_prints(4, i, "[O]");
		} else {
			tft_prints(4, i, "[X]");
		}
		
		tft_prints(8, i, "#%d:", (i<<1)+1);
		if (lm629_chip_available & (1 << (i*2+1))) {
			tft_prints(12, i, "[O]");
		} else {
			tft_prints(12, i, "[X]");
		}
	}
	
	tft_prints(2, 6, "Lock");
	tft_prints(2, 7, "Start");
	tft_prints(9, 6, "[ZERO]");
	
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			button_update();
			
			tft_prints(1, index+6, ">");
			tft_prints(tft_width-1, tft_height-1, "%d", get_seconds()%10);
	           
			if (button_down & UP || button_down & DOWN) {
				tft_prints(1, index+6, " ");  
				index = 1-index;
			} else if (button_down & START) {
				if (index == 0) {
					for (i = 0; i < MAX_CHIP; i++)
						if (lm629_chip_available & (1 << i))
							lm629_stop_abruptly(i);
					tft_prints(9, 6, "[LOCK]");
				}
			} else if (button_down & WAKEUP) {
				lm629_disable_output();
				return;
			}
			
			if (index == 1) {
				if (button_data & LEFT) {
					for (i = 0; i < MAX_CHIP; i++)
						if (lm629_chip_available & (1 << i))
							lm629_velocity_start(i, 200000);
					tft_prints(9, 6, "[<<<<]");
				} else if (button_data & RIGHT) {
					for (i = 0; i < MAX_CHIP; i++)
						if (lm629_chip_available & (1 << i))
							lm629_velocity_start(i, -200000);
					tft_prints(9, 6, "[>>>>]");
				} else {
					lm629_disable_output();
					tft_prints(9, 6, "[ZERO]");
				}
			}
			
			tft_update();
		}
	}
}

/**
  * @brief  Encoder test
  * @param  None
  * @retval None
  */
void encoder_test(void)
{
	u8 i = 0, page = 0, n = 0;

	tft_clear();
	while(1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();

			button_update();

			if (button_down & WAKEUP)
				return;

			if (!lm629_chip_available) {
				tft_prints(1, 1, "[No chips!]");
				tft_prints(tft_width-1, tft_height-1, "%d", get_seconds()%10);
				tft_update();
				continue;
			}
			
			n = 0;
			for (i = 0; i < MAX_CHIP; i++)
				if (lm629_chip_available & (1 << i))
					n++;

			if (button_down & LEFT) {
				page = (page + ((n-1)/tft_height) - 1) % ((n-1)/tft_height);
				tft_clear();
			} else if (button_down & RIGHT) {
				page = (page + 1) % ((n-1)/tft_height);
				tft_clear();
			}
			tft_prints(tft_width-1, tft_height-1, "%d", get_seconds()%10);
			
			n = 0;
			for (i = 0; i < MAX_CHIP; i++) {
				if (lm629_chip_available & (1 << i)) {
					if (n >= page*tft_height) {
						tft_clear_line(n % tft_height);
						tft_prints(1, n % tft_height, "#%d:", i);
						tft_prints(5, n % tft_height, "[%ld]", lm629_read_pos(i));
					}
					n++;
					if (n >= page*tft_height+tft_height)
						break;
				}
			}
			
			tft_update();
		}
	}
}
#endif

/**
  * @brief  Position test (gyro)
  * @param  None
  * @retval None
  */
#ifdef __GYRO_H
void position_test(void)
{
	u8 index = 0;
	u16 gyro_timeout = 0;
	
	tft_clear();
	tft_prints(2, 5, "Set Zero");
	tft_prints(2, 6, "Calibrate");
	while(1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			button_update();
			
			tft_clear_line(0);
			tft_clear_line(1);
			tft_clear_line(2);
			tft_prints(1, 1, "X: [%6d]", get_X());
			tft_prints(1, 2, "Y: [%6d]", get_Y());
			tft_prints(1, 3, "A: [%6d]", get_angle());
			tft_prints(1, index+5, ">");
			
			if (!gyro_available) {
				tft_prints(12, 5, "[!!]");
			} else {
				tft_prints(12, 5, "   ");
			}
			
			tft_prints(tft_width-1, tft_height-1, "%d", get_seconds()%10);
	
			if (button_down & WAKEUP) {
				return;
			} else if (button_down & UP || button_down & DOWN) {
				tft_prints(1, index+5, " ");
				index = 1-index;
			} else if (button_down & START) {
				if (index == 0) {
					gyro_timeout = 0;
					while (!gyro_pos_set(0, 0, 0) && gyro_timeout < 30) {
						gyro_timeout++;
					}
					if (gyro_timeout == 30) {
						buzzer_control(3, 5);
					}
				} else if (index == 1) {
					gyro_cal();
				}
			}
			tft_update();
		}
	}
}
#endif

/**
  * @brief  Servo test
  * @param  None
  * @retval None
  */
#ifdef __SERVO_H
void servo_test(void)
{
	u8 i = 0, index = 0, inited = 0;
	s16 servo_val[8];

	tft_clear();
	tft_prints(1, 1, ">Init"); 
	tft_prints(2, 3, "Warning!!!");
	tft_prints(2, 4, "Servo wil be");
	tft_prints(2, 5, "set to [50%%]");

	while (!inited) {
		button_update();
		
		tft_prints(tft_width-1, tft_height-1, "%d", get_seconds()%10);

		if (button_down & START) {
			for (i = 0; i < 4; i++) {
				servo_val[i] = 500;
				servo_control(i, servo_val[i]);
			}
			inited = 1;
		} else if (button_down & WAKEUP) {
			return;
		}
		
		tft_update();
	}

	tft_clear();
	for (i = 0; i < 4; i++) {		// MAX 4 servo
		tft_prints(2, i+1, "#%d:", i);
		tft_prints(9, i+1, "[%d]", servo_val[i]);
	}

	while(1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 8 == 0) {
				button_update();
				tft_prints(tft_width-1, tft_height-1, "%d", get_seconds()%10);
				tft_prints(1, index+1, ">");
		
				if ((button_down & UP) || (button_down & DOWN)) {
					tft_prints(1, index+1, " ");
					tft_prints(6, index+1, "  ");
					tft_prints(14, index+1, "  ");
		
					if (button_down & UP) {
						index = (index+3) % 4;
					} else {
						index = (index+1) % 4;
					}
				}
				if ((button_data & LEFT) || (button_data & RIGHT)) {
					if (button_data & LEFT) {
						tft_prints(6, index+1, "<<");
						servo_val[index] -= 5;
					} else if (button_data & RIGHT) {
						tft_prints(14, index+1, ">>");
						servo_val[index] += 5;
					}
					servo_val[index] = servo_val[index] > 1000 ? 1000 : servo_val[index] < 0 ? 0 : servo_val[index];
					tft_prints(9, index+1, "    ");
					tft_prints(9, index+1, "[%d]", servo_val[index]);
					servo_control(index, servo_val[index]);
				} else {
					tft_prints(6, index+1, "  ");
					tft_prints(14, index+1, "  ");
				}
		
				if (button_down & WAKEUP)
					return;
		
				tft_update();
			}
		}
	}
}
#endif

/**
  * @brief  PWM test
  * @param  None
  * @retval None
  */
#ifdef __MOTOR_PWM_H
void pwm_test(void)
{
	u8 i = 0, index = 0;
	s16 pwm_val[4];

	tft_clear();
	for (i = 0; i < 4; i++) {
		tft_prints(2, i, "#%d:", i);
		tft_prints(12, i, "[0]");
		pwm_val[i] = 0;
	}
	
	tft_prints(2, 5, "Hold [START]");
	tft_prints(2, 6, "to rotate");

	while(1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 4 == 0) {
			
				button_update();
				tft_prints(tft_width-1, tft_height-1, "%d", get_seconds()%10);
				tft_prints(1, index, ">");
		
				if ((button_down & UP) || (button_down & DOWN)) {
					tft_prints(1, index, " ");
					tft_prints(6, index, "  ");
					tft_prints(14, index, "  ");
					motor_control(index, 0);
					if (button_down & UP) {
						index = (index+3) % 4;
					} else {
						index = (index+1) % 4;
					}
				}
				if ((button_data & LEFT) || (button_data & RIGHT)) {
					if (button_data & LEFT) {
						tft_prints(6, index, "<<");
						pwm_val[index] -= 5;
					} else {
						tft_prints(14, index, ">>");
						pwm_val[index] += 5;
					}
					
					pwm_val[index] = pwm_val[index] > 1000 ? 1000 : pwm_val[index] < -1000 ? -1000 : pwm_val[index];
					tft_prints(8, index, "     ");
					
					tft_prints(8, index, "[%5d]", pwm_val[index]);
				} else if (button_data & START) {
					motor_control(index, pwm_val[index]);
					tft_prints(6, index, "**");
					tft_prints(14, index, "**");
				} else {
					motor_control(index, 0);
					tft_prints(6, index, "  ");
					tft_prints(14, index, "  ");
				}
		
				if (button_down & WAKEUP) {
					for (i = 0; i < 4; i++) {
						motor_control(i, 0);
					}
					return;
				}
		
				tft_update();
			}
		}
	}
}
#endif

/**
  * @brief  Getting the info of the program
  * @param  None
  * @retval None
  */
void get_program_info(void)
{	
	tft_clear();
	tft_prints(1, 1, "Last Modified:");
	tft_prints(2, 2, "["__DATE__"]");
	tft_prints(2, 3, "["__TIME__"]");
	tft_prints(1, 5, "Compiled by:");
	tft_prints(2, 6, "["COMPUTER_NAME"]");
	tft_update();
	
	while(1) {
		button_update();
		if (button_down)
			return;
		
		tft_prints(tft_width-1, tft_height-1, "%d", get_seconds()%10);
		tft_update();
	}
}

/**
  * @brief  GPIOE test
  * @param  None
  * @retval None
  */
void gpioe_test(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	u8 i, index = 0;
	u8 modes[16];
	u16 clr1 = 0, clr2 = 0;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	for (i = 0; i < 16; i++)
		modes[i] = 0;
	
	tft_clear();
	for (i = 0; i < 4; i++) {
		tft_prints(1, i+1, "#%d:", i*4);
	}
	tft_prints(1, 6, "<<");
	tft_prints(13, 6, ">>");
	
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();

			button_update();
			if (button_down & WAKEUP)
				return;
			
			if (button_down & (LEFT | RIGHT | UP | DOWN)) {
				tft_prints(6+(index%4)*2, index/4+1, " ");
				tft_prints(8+(index%4)*2, index/4+1, " ");
				if (button_down & LEFT) {
					index = (index/4)*4+(((index%4)+3)%4);
				} else if (button_down & RIGHT) {
					index = (index/4)*4+(((index%4)+1)%4);
				} else if (button_down & UP) {
					index = (((index/4)+3)%4)*4 + (index%4);
				} else if (button_down & DOWN) {
					index = (((index/4)+1)%4)*4 + (index%4);
				}
			} else if (button_down & START) {
				modes[index] = (modes[index]+1)%3;
				GPIO_InitStructure.GPIO_Pin = (1 << index);
				GPIO_InitStructure.GPIO_Mode = (modes[index] == 0 ? GPIO_Mode_IN_FLOATING : (modes[index] == 1 ? GPIO_Mode_IPU : GPIO_Mode_IPD));
				GPIO_Init(GPIOE, &GPIO_InitStructure);
			}
			
			tft_prints(6+(index%4)*2, index/4+1, ">");
			tft_prints(8+(index%4)*2, index/4+1, "<");
			
			tft_prints(tft_width-1, tft_height-1, "%d", get_seconds()%10);
			
			clr1 = curr_text_color;
			clr2 = curr_bg_color;
			for (i = 0; i < 16; i++) {
				tft_set_text_color(modes[i] == 0 ? DARK_GREY : WHITE);
				tft_set_bg_color(modes[i] == 0 ? clr2 : (modes[i] == 1 ? RED : BLUE));
				tft_prints(7+(i%4)*2, i/4+1, "%d", GPIO_ReadInputDataBit(GPIOE, (1 << i)));
			}
			tft_set_text_color(clr1);
			tft_set_bg_color(clr2);
			
			if (modes[index] == 0) {
				tft_prints(4, 6, "IN_FLOAT");
			} else if (modes[index] == 1) {
				tft_prints(4, 6, "   IPU  ");
			} else {
				tft_prints(4, 6, "   IPD  ");
			}
			
			tft_update();
		}
	}
}

/**
  * @brief  Function to be called at low voltage level
  * @param  None
  * @retval None
  */
void battery_low(void)
{
#ifdef __LM629_H
	lm629_disable_output();
#endif
#ifdef __MOTOR_PWM_H
	motor_control(0, 0);
	motor_control(1, 0);
	motor_control(2, 0);
	motor_control(3, 0);
#endif
	led_control(LED_R | LED_G | LED_B, LED_ON);
	buzzer_control(8, 5);
	_delay_ms(1000);
	led_control(LED_R | LED_G | LED_B, LED_OFF);
	_delay_ms(1000);
	while(1);
}
