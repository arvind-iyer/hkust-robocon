#include "robocon.h"

static u16 ticks_img 	= (u16)-1;

void robocon_main(void)
{
  // Send the acceleration data
	wheel_base_tx_acc();
	
	for(;;) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 10 == 0) {
				// Every 10 ms (100 Hz)
				bluetooth_update();
				wheel_base_pid_update();
				wheel_base_update();
				racket_update();
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
					wheel_base_override_set_vel(0, 100);
				}
				if (button_pressed(BUTTON_JS2_DOWN)) {
					wheel_base_override_set_vel(0, -100);
				}
				if (button_pressed(BUTTON_JS2_LEFT)) {
					wheel_base_override_set_vel(-100, 0);
				}
				if (button_pressed(BUTTON_JS2_RIGHT)) {
					wheel_base_override_set_vel(100, 0);
				}
				if (button_pressed(BUTTON_JS2_CENTER) == 1) {
					wheel_base_override_change_speed();
				}
			}
			
			if (ticks_img % 50 == 7) {
				// Every 50 ms (20 Hz)
				/** Warning: try not to do many things after tft_update(), as it takes time **/

				WHEEL_BASE_VEL vel = wheel_base_get_vel();
				tft_clear();
				draw_top_bar();

				tft_prints(0, 1, "V:(%3d,%3d,%3d)", vel.x, vel.y, vel.w);
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
        //tft_prints(0, 6, "Char: %s (%d)", s, special_char_handler_bt_get_last_char());
				
				//tft_prints(0, 6, "%d [%d] %d", get_kpx()/10, get_kix()/10);
				//tft_prints(0, 7, "[%d] %d", get_vel_limit(), get_kdx()/10);
				

				tft_prints(0, 6, "B%s", get_b1()==1?"[1]":"1");
				tft_prints(2, 6, "%s", get_b2()==1?"[2]":"2");
				tft_prints(3, 6, "%s", get_b3()==1?"[3]":"3");
				tft_prints(0, 7, "S%s", get_s1()==1?"[1]":"1");
				tft_prints(2, 7, "%s", get_s2()==1?"[2]":"2");
				
				tft_prints(4, 6, "  UP [%d]", get_high_speed());
				tft_prints(4, 7, " LOW [%d]", get_low_speed());
				
				tft_prints(0, 8, "[%d %d %d]", wheel_base_pid_current_vel().x, wheel_base_pid_current_vel().y, wheel_base_pid_current_vel().w);
				tft_prints(0, 9, "[%d %d %d]", wheel_base_pid_unlimit_vel().x, wheel_base_pid_unlimit_vel().y, wheel_base_pid_unlimit_vel().w);
				tft_update();
				
				NVIC_EnableIRQ(EXTI15_10_IRQn);
			}
		}
	}	
}
