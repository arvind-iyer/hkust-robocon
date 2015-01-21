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

