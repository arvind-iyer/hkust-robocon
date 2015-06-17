#include "robocon.h"

static u16 ticks_img 	= (u16)-1;

void robocon_main(void) {
  // Send the acceleration data
	wheel_base_tx_acc();

	for (; ; ) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			
			if (ticks_img % 10 == 0) {
				// Every 10 ms (100 Hz)
				bluetooth_update();
				button_update();
				button_event_update();
				wheel_base_pid_update();
			wheel_base_update();
				racket_update();
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
			}
			
			if (ticks_img % 50 == 7) {
				// Every 50 ms (20 Hz)
				/** Warning: try not to do many things after tft_update(), as it takes time **/

				WHEEL_BASE_VEL vel = wheel_base_get_vel();
				tft_clear();
				//draw_top_bar();

				tft_prints(0, 1, "V:(%3d,%3d,%3d)", vel.x, vel.y, vel.w);
				tft_prints(0, 2, "Speed: %d", wheel_base_get_speed_mode());
				tft_prints(0, 3, "(%-4d,%-4d,%-4d)", get_pos()->x, get_pos()->y,get_pos()->angle);
				//tft_prints(0, 4, "%s", button_event_get_side_control() == SIDE_NORMAL ? "NORMAL CTRL" : 
				//	(button_event_get_side_control() == SIDE_RIGHT ? "[RIGHT SIDE CTRL]" : "[LEFT SIDE CTRL]")
				//);
				
				tft_prints(0, 5, "%s", is_force_terminate() ? "   FORCE" : "");
				tft_prints(0, 7, "(%4d)  F (%4d)", wheel_base_get_vel_top_left(), wheel_base_get_vel_top_right());
				tft_prints(0, 8, "  L  <[%4d]>  R ", button_event_get_l_analog_magnitude());
				tft_prints(0, 9, "(%4d) B  (%4d)", wheel_base_get_vel_bottom_left(), wheel_base_get_vel_bottom_right());
				tft_update();
			}
		}
	}	
}
