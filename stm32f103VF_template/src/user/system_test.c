#include "system_test.h"

static u16 ticks_img 	= (u16)-1;
static u16 seconds_img = (u16)-1;

static const char SELECTED = (char) 127;

static u8 return_listener(void)
{
	return button_pressed(BUTTON_1) > 10 || button_pressed(BUTTON_2) > 10;
}

void battery_test(void)
{
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					return; 
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "BATTERY TEST");
				tft_prints(0, 2, "ADC: %d", get_battery_adc());
				tft_prints(0, 3, "V: %d", get_voltage());
				tft_prints(0, 4, "V avg: %d", get_voltage_avg());
				tft_update();
			}
		}
	}
}

void bluetooth_test(void)
{
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					return; 
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				
				char c = 0;
				
				tft_prints(0, 1, "Received: %d", bluetooth_get_data_count());
				tft_prints(0, 2, "Recent data: ");
				tft_prints(0, 3, "ID: 0x%X", bluetooth_recent_rx_id());
				tft_prints(0, 4, "Length: %d", bluetooth_recent_rx_data_length());
				tft_prints(0, 5, "Data: ");
				const u8* data = bluetooth_recent_rx_data();
				tft_prints(0, 6, "{%02X,%02X,%02X,%02X,", data[0], data[1], data[2], data[3]);
				tft_prints(0, 7, " %02X,%02X,%02X,%02X}", data[4], data[5], data[6], data[7]);
				tft_update();
			}
		}
	}		
}

void ascii_test(void)
{
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					return; 
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				
				char c = 0;
				
				/** Display characters from ascii value 0 **/
				for (u8 y = 1; y < tft_height; ++y) {
					for (u8 x = 0; x < tft_width; ++x) {
						++c;
						tft_prints(x, y, "%c", c);
						
					}
				}	
				
				tft_update();
			}
		}
	}	
}

void motor_test(void)
{
	MOTOR_ID motor_id = 0;
	const u8 line_available[] = {2, 5, 6, 7};
	u8 line_id = 0;
	CLOSE_LOOP_FLAG close_loop_flag = OPEN_LOOP;
	s32 speed = 0;
	u8 test_flag = 0;
	
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			u8 line = line_available[line_id];
			

			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					return; 
				}
				
				// Line switcher
				if (button_pressed(BUTTON_JS2_UP) == 1) {
					if (line_id == 0) {
						line_id = sizeof(line_available) / sizeof(u8) - 1;
					} else {
						--line_id;
					}
				}
				
				if (button_pressed(BUTTON_JS2_DOWN) == 1) {
					line_id = (line_id + 1) % (sizeof(line_available) / sizeof(u8));
				}
				// Line 2: Motor selector
				if (line == 2) {
					if (button_pressed(BUTTON_JS2_LEFT) == 1) {
						if (motor_id > 0) {
							--motor_id;
							CLICK_MUSIC;
						}
					}
					
					if (button_pressed(BUTTON_JS2_RIGHT) == 1) {
						if (motor_id < CAN_MOTOR_COUNT - 1) {
							++motor_id;
							CLICK_MUSIC;
						}
					}
				}
				
				// Line 5: Close loop flag switcher
				if (line == 5) {
					if (button_pressed(BUTTON_JS2_LEFT) == 1 || button_pressed(BUTTON_JS2_RIGHT) == 1) {
						close_loop_flag = !close_loop_flag;
						CLICK_MUSIC;
					}
				}
				
				// Line 6: Speed
				if (line == 6) {
					if (button_pressed(BUTTON_JS2_LEFT) == 1 || button_hold(BUTTON_JS2_LEFT, 10, 1)) {
						--speed;
					}
					if (button_pressed(BUTTON_JS2_RIGHT) == 1 || button_hold(BUTTON_JS2_RIGHT, 10, 1)) {
						++speed;
					}
				}
				
				if (line == 7) {
					if (button_pressed(BUTTON_JS2_CENTER) == 1) {
						test_flag = !test_flag;
						CLICK_MUSIC;
					}
				}
			}
			
			if (ticks_img % 50 == 5) {
				if (test_flag) {
					// Start setting velocity
					motor_set_vel(motor_id, speed, close_loop_flag);
				} else {
					// Stop
					motor_set_vel(motor_id, 0, OPEN_LOOP);
				}
			}
			
			if (ticks_img % 50 == 7) {
				tft_clear();
				draw_top_bar();
				
				tft_prints(0, 1, "MOTOR TEST");
				tft_prints(0, 2, "  %c  MOTOR%-2d %c", (ticks_img < 500 && line == 2) ? SELECTED : '<', \
					motor_id, (ticks_img < 500 && line == 2) ? SELECTED : '>');
				tft_prints(0, 3, "ID: 0x%03X", CAN_MOTOR_BASE + motor_id); 
				tft_prints(0, 4, "Encoder: %d", get_encoder_value(motor_id));
				
				tft_prints(0, 5, "  %c   %-5s  %c", (ticks_img < 500 && line == 5) ? SELECTED : '<', \
					close_loop_flag == OPEN_LOOP ? "OPEN" : "CLOSE", (ticks_img < 500 && line == 5) ? SELECTED : '>');
				
				tft_prints(0, 6, "Speed: ");
				if (ticks_img < 500 && line == 6) {	// Flashing the number
					tft_prints(7, 6, "%d", speed);
				} 
				tft_prints(0, 7, " (%c) Start test", (ticks_img < 500 && line == 7) ? SELECTED : (test_flag ? 'X' : ' '));
				
				tft_update();
			}
		}
	}		
}

void position_test(void)
{
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					return; 
				}
				
				if (button_pressed(BUTTON_JS2_CENTER) == 1) {
					gyro_cal();
					CLICK_MUSIC;
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "POSITION TEST");
				tft_prints(0, 2, " X: %d", get_pos()->x);
				tft_prints(0, 3, " Y: %d", get_pos()->y);
				tft_prints(0, 4, " A: %d", get_pos()->angle);
				tft_prints(0, 5, " Avail: %d", gyro_available);
				
				tft_prints(0, 7, " (%c) Calibrate", ticks_img < 500 ? SELECTED : ' ');
				
				tft_update();
			}
		}
	}
}

void button_test(void)
{
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					return; 
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "BUTTON TEST");
				for (u8 i = 0, x = 1, y = 2; i < BUTTON_COUNT; ++i) {
					tft_prints(x, y, "%d:%d", i, button_pressed(i));
					if (y < tft_height - 1) {
						++y;
					} else {
						x += 6;
						y = 2;
					}
				}
				tft_update();
			}
		}
	}	
}