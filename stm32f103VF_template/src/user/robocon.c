#include "robocon.h"

static u16 ticks_img 	= (u16)-1;

void robocon_main(void)
{
  // Send the acceleration data
	wheel_base_tx_acc();
	
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
		
			// 1000 Hz
			up_racket_sensor_check();
			
			if (ticks_img % 10 == 0) {
				// Every 10 ms (100 Hz)
				bluetooth_update();
				xbc_button_handler();
        wheel_base_pid_update();
				wheel_base_update();
				racket_update();
				up_racket_update();   
			}
			
			if (ticks_img % 250 == 1) {
				// Every 250 ms (4 Hz)
				battery_adc_update();
			}
			
			if (get_seconds() % 10 == 2 && ticks_img == 2) {
				// Every 10 seconds (0.1 Hz)
				battery_regular_check();
			}

			if (ticks_img % 100 == 3) {
				// Every 100 ms (10 Hz)
				wheel_base_tx_position();
			}
			
			if (ticks_img % 500 == 4) {
				led_control(LED_D3, (LED_STATE) (ticks_img == 0));
			}			
			if (ticks_img % 50 == 5) {
				button_update();
				if (button_pressed(BUTTON_1) > 10 || button_pressed(BUTTON_2) > 10) {
					/** Stop the wheel base before return **/
					break; 
				}
				if (button_pressed(BUTTON_JS2_UP)) {
					wheel_base_override_set_vel(0, 30, 0);
				}
				if (button_pressed(BUTTON_JS2_DOWN)) {
					wheel_base_override_set_vel(0, -30, 0);
				}
				if (button_pressed(BUTTON_JS2_LEFT)) {
					wheel_base_override_set_vel(-30, 0, 0);
				}
				if (button_pressed(BUTTON_JS2_RIGHT)) {
					wheel_base_override_set_vel(30, 0, 0);
				}
				if (button_pressed(BUTTON_JS2_CENTER) == 1) {
					wheel_base_override_change_speed();
				}
			}
			
			if (ticks_img % 50 == 7) {
				// Every 50 ms (20 Hz)
				/** Warning: try not to do many things after tft_update(), as it takes time **/

				WHEEL_BASE_VEL vel = wheel_base_get_vel();
				WHEEL_BASE_VEL vel_prev = wheel_base_get_prev_vel();
				tft_clear();
				draw_top_bar();

				tft_prints(0, 1, "V:(%3d,%3d,%3d)", vel.x / 100, vel.y / 100, vel.w / 100);
				tft_prints(0, 2, "Speed: %d", wheel_base_get_speed_mode());
				tft_prints(0, 3, "(%-4d,%-4d,%-4d)", get_pos()->x, get_pos()->y,get_pos()->angle);
				tft_prints(0, 4, "%s", wheel_base_get_pid_flag() ? "[AUTO]" : "MANUAL");
				tft_prints(0, 5, "(%-4d,%-4d,%-4d)", wheel_base_get_target_pos().x, wheel_base_get_target_pos().y, wheel_base_get_target_pos().angle);
				char s[3] = {special_char_handler_bt_get_last_char(), '\0'};
        if (s[0] == '[' || s[0] == ']') {
          // Replace "[" and "]" as "\[" and "\]"
          s[1] = s[0];
          s[0] = '\\';
        }
        tft_prints(0, 6, "Char: %s (%d)", s, special_char_handler_bt_get_last_char());
        //tft_prints(0, 7, "Switch hit: %d", did_receive_command());
				//tft_prints(0, 8, "switch: %d",get_switch());
				tft_prints(0, 7, "T:(%3d,%3d,%3d)", get_prop(), get_int(), get_der());
				tft_prints(0, 8, "VP(%3d,%3d,%3d)", vel_prev.x / 100, vel_prev.y / 100, vel_prev.w / 100);
				tft_prints(0, 9, "D: %3d, S: %4d", get_racket_delay(), get_racket_speed());
//				tft_prints(0, 9, "PID locked: %d", get_pid_stat());

				tft_update();
			}
		}
	}	
}
