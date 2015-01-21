#include "robocon.h"

static u16 ticks_img 	= (u16)-1;
static u16 seconds_img = (u16)-1;

void robocon_main(void)
{
	wheel_base_tx_acc();
	
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 10 == 0) {
				// Every 10 ms (100 Hz)
				bluetooth_update();
				wheel_base_update();
				wheel_base_pid_loop();
			}
			
			if (ticks_img % 250 == 1) {
				// Every 250 ms (4 Hz)
				battery_adc_update();
			}
			
			if (get_seconds() % 10 == 0 && ticks_img == 2) {
				// Every 10 seconds (0.1 Hz)
				battery_regular_check();
			}

			
			if (ticks_img % 100 == 3) {
				// Every 100 ms (10 Hz)
				wheel_base_tx_position();
			}
			
			if (ticks_img % 500 == 0) {
				led_control(LED_D3, (LED_STATE) (ticks_img == 0));
			}
			
			if (ticks_img % 50 == 7) {
				// Every 50 ms (20 Hz)
				/** Warning: try not to do many things after tft_update(), as it takes time **/
				button_update();
				
				WHEEL_BASE_VEL vel = wheel_base_get_vel();
				tft_clear();
				tft_prints(0, 0, "Battery: %s", get_voltage_string());
				tft_prints(0, 1, "ADC: %d", get_battery_adc());
				tft_prints(0, 2, "VX: %d", vel.x);
				tft_prints(0, 3, "VY: %d", vel.y);
				tft_prints(0, 4, "VW: %d", vel.w);
				tft_prints(0, 5, "Speed: %d", wheel_base_get_speed_mode());
				tft_set_bg_color(BLACK);
				tft_clear_line(5);
				tft_set_bg_color(WHITE);
				tft_prints(0, 6, "(%-4d,%-4d,%-4d)", get_pos()->x, get_pos()->y,get_pos()->angle);
				tft_prints(0, 7, "%d%d%d%d%d%d%d%d%d%d", \
				button_pressed(BUTTON_JS1_UP) % 10, button_pressed(BUTTON_JS1_DOWN) % 10, button_pressed(BUTTON_JS1_LEFT) % 10, button_pressed(BUTTON_JS1_RIGHT) % 10, button_pressed(BUTTON_JS1_CENTER) % 10, \
				button_pressed(BUTTON_JS2_UP) % 10, button_pressed(BUTTON_JS2_DOWN) % 10, button_pressed(BUTTON_JS2_LEFT) % 10, button_pressed(BUTTON_JS2_RIGHT) % 10, button_pressed(BUTTON_JS2_CENTER) % 10);
				tft_prints(0, 8, "%d%d%d%d%d%d%d%d%d%d", \
				button_released(BUTTON_JS1_UP) % 10, button_released(BUTTON_JS1_DOWN) % 10, button_released(BUTTON_JS1_LEFT) % 10, button_released(BUTTON_JS1_RIGHT) % 10, button_released(BUTTON_JS1_CENTER) % 10, \
				button_released(BUTTON_JS2_UP) % 10, button_released(BUTTON_JS2_DOWN) % 10, button_released(BUTTON_JS2_LEFT) % 10, button_released(BUTTON_JS2_RIGHT) % 10, button_released(BUTTON_JS2_CENTER) % 10);

				tft_prints(0, 9, "Time: %d'%03d\"", get_seconds(), get_ticks());
				tft_update();
			}
			
			
		}
	}	
}
