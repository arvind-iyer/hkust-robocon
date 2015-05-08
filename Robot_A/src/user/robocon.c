#include "robocon.h"
#include "upper_racket.h"
#include "serving.h"

static u16 ticks_img 	= (u16)-1;

/**
	* @brief Process XBC input
	*/
static void robocon_get_xbc(void)
{
	// Processing wheel base velocity
	u8 speed_mode = wheel_base_get_speed_mode();
	u16 speed_ratio = SPEED_MODES[speed_mode];
	
	s16 x_vel = xbc_get_joy(XBC_JOY_LX);
	s16 y_vel = xbc_get_joy(XBC_JOY_LY);
	s16 w_vel = xbc_get_joy(XBC_JOY_RT) - xbc_get_joy(XBC_JOY_LT);
	
	x_vel = speed_ratio * x_vel / 1000;
	y_vel = speed_ratio * y_vel / 1000;
	w_vel = speed_ratio * w_vel / 255 / 2;
	wheel_base_set_vel(x_vel, y_vel, w_vel);
	
	if (button_pressed(BUTTON_XBC_L_JOY) == 1) {
		if (speed_mode > 0) {
			wheel_base_set_speed_mode(speed_mode-1);
			buzzer_control_note(1, 100, NOTE_C, 7);
		}
	}
	if (button_pressed(BUTTON_XBC_R_JOY) == 1) {
		if (speed_mode < sizeof(SPEED_MODES) / sizeof(u16) - 1) {
			wheel_base_set_speed_mode(speed_mode+1);
			buzzer_control_note(1, 100, NOTE_C, 8);
		}
	}
	
	if (button_pressed(BUTTON_XBC_Y) == 1) {
		upper_racket_hit(0);
	}
	
	if (button_pressed(BUTTON_XBC_B) == 1) {
		if (get_serving_calibrated()) {
			serving_hit_start();
		} else {
			buzzer_control_note(3, 100, NOTE_G, 5);
		}
	}
	
	if (button_pressed(BUTTON_XBC_B) == 40 && !get_serving_calibrated()) {
		serving_hit_start();
	}
	
	if (button_pressed(BUTTON_XBC_X) == 1) {
		serving_cali_start();
	}
	
	if (button_pressed(BUTTON_XBC_W) == 1 || button_hold(BUTTON_XBC_W, 20, 1)) {
		u16 delay = get_shuttle_drop_delay();
		if (delay > 1) {
			set_shuttle_drop_delay(delay - 1);
			buzzer_control_note(1, 10, NOTE_C, 6);
		}
	}
	
	if (button_pressed(BUTTON_XBC_E) == 1 || button_hold(BUTTON_XBC_E, 20, 1)) {
		u16 delay = get_shuttle_drop_delay();
		if (delay < 1000) {
			set_shuttle_drop_delay(delay + 1);
			buzzer_control_note(1, 10, NOTE_C, 7);
		}
	}
	
	if (button_pressed(BUTTON_XBC_S) == 1 || button_hold(BUTTON_XBC_S, 20, 1)) {
		s16 speed = get_serving_hit_speed();
		if (speed > -1800) {
			set_serving_hit_speed(speed - 10);
			buzzer_control_note(1, 10, NOTE_E, 6);
		}
	}
	
	if (button_pressed(BUTTON_XBC_N) == 1 || button_hold(BUTTON_XBC_N, 20, 1)) {
		s16 speed = get_serving_hit_speed();
		if (speed < 1800) {
			set_serving_hit_speed(speed + 10);
			buzzer_control_note(1, 10, NOTE_E, 7);
		}
	}
}


void robocon_main(void)
{
	upper_racket_init();
	serving_init();
	
  // Send the acceleration data
	wheel_base_tx_acc();
	
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 10 == 0) {
				// Every 10 ms (100 Hz)
				bluetooth_update();
        wheel_base_pid_update();
				wheel_base_update();
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
			
			if (ticks_img % 20 == 5) {		// 50Hz
				button_update();
				robocon_get_xbc();
				serving_update();
				upper_racket_update();
				
				
				if (return_listener()) {
					/** Stop the wheel base before return **/
					wheel_base_stop();
					return; 
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
				tft_prints(0, 4, "State: (%d,%d)", get_serving_cali_state(), get_serving_hit_state());
				tft_prints(0, 5, get_serving_calibrated() ? "CALI" : "[NOT CAL!]"); 
				tft_prints(10, 5, get_serving_switch() ? "SW": "--");
				s32 target_encoder = 0;
				if (get_serving_cali_state() != SERVING_CALI_NULL) {
					s32 target_encoder = get_serving_cali_encoder_target();
				} else if (get_serving_hit_state() != SERVING_NULL) {
					s32 target_encoder = get_serving_hit_encoder_target();
				}
				tft_prints(0, 6, " %d->%d", get_serving_encoder(), get_serving_hit_encoder_target());
				
				tft_prints(0, 7, "Delay: %d", get_shuttle_drop_delay());
				
				tft_prints(0, 8, "Force: %d", get_serving_hit_speed());
				
				//tft_prints(0, 4, "%s", wheel_base_get_pid_flag() ? "[AUTO]" : "MANUAL");
				/*tft_prints(0, 5, "(%-4d,%-4d,%-4d)", wheel_base_get_target_pos().x, wheel_base_get_target_pos().y, wheel_base_get_target_pos().angle);
				char s[3] = {wheel_base_bluetooth_get_last_char(), '\0'};
        if (s[0] == '[' || s[0] == ']') {
          // Replace "[" and "]" as "\[" and "\]"
          s[1] = s[0];
          s[0] = '\\';
        }
        tft_prints(0, 6, "Char: %s (%d)", s, wheel_base_bluetooth_get_last_char());
        */
				
				tft_update();
			}
			
			
		}
	}	
}
