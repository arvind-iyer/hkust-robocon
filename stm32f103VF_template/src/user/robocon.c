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
				pivot_update();
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
				tft_prints(0, 1, "1_%d,%d,%d,", get_feedback(0,0),get_feedback(0,1),get_feedback(0,2));
				tft_prints(0, 2, "2_%d,%d,%d", get_feedback(1,0),get_feedback(1,1),get_feedback(1,2));
				tft_prints(0, 3, "YT_%d,%d,%d", get_feedback(2,0),get_feedback(2,1),get_feedback(2,2));
				tft_prints(0, 4, "GT_%d,%d,%d", get_feedback(3,0),get_feedback(3,1),get_feedback(3,2));
				tft_prints(0, 5, "WT_%d,%d,%d", get_feedback(4,0),get_feedback(4,1),get_feedback(4,2));
				tft_prints(0, 6, "6_%d,%d,%d", get_feedback(5,0),get_feedback(5,1),get_feedback(5,2));
				tft_prints(0, 7, "COL_%c,%c,%c,%c" ,get_color(1),get_color(2),get_color(3),get_color(4));
				tft_prints(0, 8, "COL_%c,%c,%c,%c,%c", get_color(5),get_color(6),get_color(7),get_color(8),get_color(9));
				tft_prints(0, 9, "COL_%c,%c,%c,%c,%c", get_color(10),get_color(11),get_color(12),get_color(13),get_color(14));
//				tft_prints(0, 7, "COL_%c,%c,%c,%c,%c", get_color_val(0),get_color_val(1),get_color(2),get_color(3),get_color(4));
//				tft_prints(0, 8, "COL_%c,%c,%c,%c,%c", get_color_val(5),get_color_val(6),get_color(7),get_color(8),get_color(9));
//				tft_prints(0, 9, "COL_%c,%c,%c,%c,%c", get_color_val(10),get_color_val(11),get_color(12),get_color(13),get_color(14));

			

				
				tft_update();
				NVIC_EnableIRQ(EXTI15_10_IRQn);
			}
		}
	}	
}
