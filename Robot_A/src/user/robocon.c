#include "robocon.h"
#include "upper_racket.h"
#include "serving.h"
#include "us_auto.h"
#include "angle_lock.h"
#include "flash.h"
#include "approx_math.h"
#include "angle_variance.h"
#include "adc_app.h"
#include "net_sensor.h"
#include <stdbool.h>

static u16 ticks_img 	= (u16)-1;
static bool serving_mode = false;
static u8 serving_id = 0;
static bool us_auto_mode = false;
static bool angle_lock_mode = false;
static bool abs_angle_mode = false;
static bool auto_serve_mode = false;
static bool e_stop_mode = false;

static u32 auto_serve_time = 0;

static s32 xbc_joy_scale(s32 val)
{
	if (val >= 0) {
		return val * val / XBC_JOY_SCALE;
	} else {
		return -val * val / XBC_JOY_SCALE;
	}
	//return val * val * val / XBC_JOY_SCALE / XBC_JOY_SCALE;
}


/**
	* @brief Process XBC input
	*/
static void robocon_get_xbc(void)
{
	// Processing wheel base velocity
	u8 speed_mode = wheel_base_get_speed_mode();
	
	if (get_serving_hit_state() >= SERVING_PRE_DELAY && get_serving_hit_state() <= SERVING_RACKET_HITTING) {
		// Stop mode during serve
		speed_mode = 0;
	} else if (button_pressed(BUTTON_XBC_RB) > 2) { 
		// Slow speed mode
		speed_mode = 1;
	}
	
	u16 speed_ratio = SPEED_MODES[speed_mode];
	
	
	
	s32 x_vel = xbc_joy_scale(xbc_get_joy(XBC_JOY_LX));
	s32 y_vel = xbc_joy_scale(xbc_get_joy(XBC_JOY_LY));
	
	/*
	if (abs_angle_mode) {
		xy_rotate((s32*) &x_vel, (s32*) &y_vel, get_pos()->angle);
	}
	*/
	
	s16 w_vel = xbc_get_joy(XBC_JOY_RT) - xbc_get_joy(XBC_JOY_LT);
	
	
	
	//s32 scalar = Sqrt(x_vel * x_vel + y_vel * y_vel);
	//s32 scalar_max = (XBC_JOY_SCALE * 14142) / 10000;		// Sqrt(2) divided
	
	x_vel = speed_ratio * x_vel / XBC_JOY_SCALE;
	y_vel = speed_ratio * y_vel / XBC_JOY_SCALE;
	w_vel = speed_ratio * w_vel / 255 / 2;
	
	// Angle PID later
	if (angle_lock_mode) {
		if (w_vel == 0) {
			// Angle lock
			w_vel = angle_lock_get_mv();
			// Limit the angle pid
			if (w_vel > 300) {
				w_vel = 300;
			} else if (w_vel < -300) {
				w_vel = -300;
			}
		} else {
			// Ignore the angle lock for a while
			angle_lock_ignore(500);
		}
	}
	
	/*** Sensor, stop robot moving forward when object is closed ***/
	u32 sensor_val = get_sensor();
	s32 current_y_vel = wheel_base_get_vel_real().y;
	
	if (y_vel > 0) {
		if (sensor_val >= 1500 && sensor_val < 2000) {
			if (current_y_vel > 1500 && y_vel > 1500) {
				y_vel = 1500;
			} else if (current_y_vel > 1000 && current_y_vel < 1500 && y_vel > 1000) {
				y_vel = 1000;
			}
		} else if (sensor_val >= 1000 && sensor_val < 1500) {
			if (current_y_vel > 800 && y_vel > 800) {
				//y_vel
				y_vel = 800;
			}
		} else if (sensor_val >= 600 && sensor_val < 1000) {
			if (current_y_vel > 500 && y_vel > 500) {
				y_vel = 500;
			}
			
		} else if (sensor_val >= 100 && sensor_val < 600) {
			y_vel = 10;
		}
	}
	
	#warning
	wheel_base_stop();
	if (e_stop_mode) {	
		#warning
		//wheel_base_set_vel(0, 0, 0);
		buzzer_set_note_period(get_note_period(NOTE_C, 8) + get_ticks());
		buzzer_control(1, 100);
	} else {
		//wheel_base_set_vel(x_vel, y_vel, w_vel);
	}
	
	
	/*** Change speed mode ***/
	static s16 rjx_val_prev = 0;
	s16 rjx_val = xbc_get_joy(XBC_JOY_RX);
	
	if (rjx_val_prev != -XBC_JOY_SCALE && rjx_val == -XBC_JOY_SCALE) {
		if (speed_mode > 0) {
			wheel_base_set_speed_mode(speed_mode-1);
			buzzer_control_note(1, 100, NOTE_C, 7);
		}
	}
	
	if (rjx_val_prev != XBC_JOY_SCALE && rjx_val == XBC_JOY_SCALE) {
		if (speed_mode < sizeof(SPEED_MODES) / sizeof(u16) - 1) {
			wheel_base_set_speed_mode(speed_mode+1);
			buzzer_control_note(1, 100, NOTE_C, 8);
		}
	}
	
	rjx_val_prev = rjx_val;
	
	/*** Change angle mode ***/
	static s16 rjy_val_prev = 0;
	s16 rjy_val = xbc_get_joy(XBC_JOY_RY);
	
	if (rjy_val_prev != -XBC_JOY_SCALE && rjy_val == -XBC_JOY_SCALE) {
		/*
		if (!abs_angle_mode) {
			abs_angle_mode = true;
		} else {
			gyro_pos_set(get_pos()->x, get_pos()->y, 0);
			angle_lock_ignore(200);
		}
		buzzer_control_note(2, 100, NOTE_C, 8);
		*/
	}
	
	if (rjy_val_prev != XBC_JOY_SCALE && rjy_val == XBC_JOY_SCALE) {
		/*
		abs_angle_mode = false;
		buzzer_control_note(2, 100, NOTE_C, 7);
		*/
	}
	
	rjy_val_prev = rjy_val;
	
	
	
	
	if (button_pressed(BUTTON_XBC_Y) == 1) {
		if (!serving_mode) {
			CLICK_MUSIC;
			upper_racket_hit(0);
		} else {
			buzzer_control_note(3, 100, NOTE_G, 6);
		}
	}
	
	if (button_pressed(BUTTON_XBC_Y) == 20) {
		CLICK_MUSIC;
		upper_racket_hit(0);
	}
	
	
	// Serve
	if (button_pressed(BUTTON_XBC_B) == 1) {
		if (get_serving_calibrated()) {
			CLICK_MUSIC;
			serving_mode = false;
			serving_hit_start();
		} else {
			buzzer_control_note(3, 100, NOTE_G, 5);
		}
	} else if (button_pressed(BUTTON_XBC_B) == 40 && !get_serving_calibrated()) {
		CLICK_MUSIC;
		serving_mode = false;
		serving_hit_start();
	}
	
	// Calibrate
	if (button_pressed(BUTTON_XBC_X) == 1 || button_pressed(BUTTON_CALIBRATE) == 5) {
		CLICK_MUSIC;
		serving_mode = serving_cali_start();
	}
	
	// Auto-serve
	if (button_pressed(BUTTON_AUTO_SERVE) == 5) {
		if (auto_serve_mode) {
			// Off
			auto_serve_mode = false;
			auto_serve_time = 0;
			buzzer_control_note(1, 500, NOTE_D, 6);
		} else {
			// On
			auto_serve_mode = true;
			auto_serve_time = get_full_ticks();
			buzzer_control_note(1, 500, NOTE_D, 7);
		}
	}
	
	/*** Serve set ***/
	if (button_pressed(BUTTON_XBC_LB) == 1) {
		// Switch serving set
		set_serving_set(!get_serving_set());
		CLICK_MUSIC;
	}
	
	
	/*** Shuttle drop delay -- ***/
	if (button_pressed(BUTTON_XBC_W) == 1 || button_hold(BUTTON_XBC_W, 20, 1)) {
		u16 delay = get_shuttle_drop_delay();
		if (delay > 1) {
			set_shuttle_drop_delay(delay - 1);
			buzzer_control_note(1, 10, NOTE_C, 6);
		}
	}
	
	/*** Shuttle drop delay ++ ***/
	if (button_pressed(BUTTON_XBC_E) == 1 || button_hold(BUTTON_XBC_E, 20, 1)) {
		u16 delay = get_shuttle_drop_delay();
		if (delay < 1000) {
			set_shuttle_drop_delay(delay + 1);
			buzzer_control_note(1, 10, NOTE_C, 7);
		}
	}
	
	/*** Decrease serving speed ***/
	if (button_pressed(BUTTON_XBC_S) == 1 || button_hold(BUTTON_XBC_S, 20, 1)) {
		s16 speed = get_serving_hit_speed();
		if (speed > -1800) {
			set_serving_hit_speed(speed - 10);
			buzzer_control_note(1, 10, NOTE_E, 6);
		}
	}
	
	/*** Increase serving speed ***/
	if (button_pressed(BUTTON_XBC_N) == 1 || button_hold(BUTTON_XBC_N, 20, 1)) {
		s16 speed = get_serving_hit_speed();
		if (speed < 1800) {
			set_serving_hit_speed(speed + 10);
			buzzer_control_note(1, 10, NOTE_E, 7);
		}
	}
	
	/*** Ultrasonic auto mode toggle ***/
	if (button_pressed(BUTTON_XBC_XBOX) == 1) {
		CLICK_MUSIC;
		//us_auto_mode = !us_auto_mode;
		e_stop_mode = !e_stop_mode;
	}
	
	/*** Anlge lock toggle ***/
	if (button_pressed(BUTTON_XBC_START) == 1) {
		CLICK_MUSIC;
		angle_lock_mode = !angle_lock_mode; 
	}
	
	if (button_pressed(BUTTON_XBC_A) == 12) {
		// Uncalibrate
		serving_uncalibrate();
		serving_mode = false;
		buzzer_control_note(3, 100, NOTE_G, 7);
	}

	
	/*** Auto serve button ***/
	
	/*** SAVE ***/
	if (button_pressed(BUTTON_XBC_R_JOY) == 10) {
		write_flash(FLASH_SHUTTLE_DROP_DELAY_OFFSET, get_shuttle_drop_delay());
		write_flash(FLASH_SERVING_HIT_SPEED_OFFSET, get_serving_hit_speed());
		write_flash(FLASH_WHEEL_BASE_SPEED_MODE_OFFSET, wheel_base_get_speed_mode());
		write_flash(FLASH_ABS_ANGLE_MODE_OFFSET, abs_angle_mode); 
		write_flash(FLASH_ANGLE_LOCK_MODE_OFFSET, angle_lock_mode);
		
		u8 tmp_set = get_serving_set();
		write_flash(FLASH_SERVING_SET_OFFSET, tmp_set);
		
		set_serving_set(0);
		write_flash(FLASH_SHUTTLE_DROP_DELAY_OFFSET, get_shuttle_drop_delay());
		write_flash(FLASH_SERVING_HIT_SPEED_OFFSET, get_serving_hit_speed());

		set_serving_set(1);
		write_flash(FLASH_SHUTTLE_DROP_DELAY_2_OFFSET, get_shuttle_drop_delay());
		write_flash(FLASH_SERVING_HIT_SPEED_2_OFFSET, get_serving_hit_speed());
		
		set_serving_set(tmp_set);
		
		buzzer_control_note(5, 100, NOTE_C, 8);
	}
}

void robocon_init(void)
{
	upper_racket_init();
	serving_init();
	
  // Send the acceleration data
	wheel_base_tx_acc();
	
	gpio_init(SERVING_LED_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
	gpio_init(US_AUTO_LED_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
	//gpio_init(ANGLE_LOCK_LED_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
	gpio_init(AUTO_SERVE_LED_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
	gpio_init(GYRO_LED_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
	gpio_init(UPPER_RACKET_GPIO, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
	
	gpio_write(SERVING_LED_GPIO, (BitAction) 1);
	gpio_write(US_AUTO_LED_GPIO, (BitAction) 1);
	//gpio_write(ANGLE_LOCK_LED_GPIO, 1);
	gpio_write(AUTO_SERVE_LED_GPIO, (BitAction) 1);
	gpio_write(GYRO_LED_GPIO, (BitAction) 1);
	gpio_write(UPPER_RACKET_GPIO, (BitAction) 1);
	
	
	serving_mode = false;
	us_auto_mode = false;
	angle_lock_mode = false;
	abs_angle_mode = false;
	
	// Get flash
	set_serving_set(0);
	if (read_flash(FLASH_SHUTTLE_DROP_DELAY_OFFSET) != -1) {
		set_shuttle_drop_delay(read_flash(FLASH_SHUTTLE_DROP_DELAY_OFFSET));
	}
	
	if (read_flash(FLASH_SERVING_HIT_SPEED_OFFSET) != -1) {
		set_serving_hit_speed(read_flash(FLASH_SERVING_HIT_SPEED_OFFSET));
	}
	
	set_serving_set(1);
	if (read_flash(FLASH_SHUTTLE_DROP_DELAY_2_OFFSET) != -1) {
		set_shuttle_drop_delay(read_flash(FLASH_SHUTTLE_DROP_DELAY_2_OFFSET));
	}
	
	if (read_flash(FLASH_SERVING_HIT_SPEED_2_OFFSET) != -1) {
		set_serving_hit_speed(read_flash(FLASH_SERVING_HIT_SPEED_2_OFFSET));
	}

	if (read_flash(FLASH_SERVING_SET_OFFSET) != -1) {
		set_serving_set(read_flash(FLASH_SERVING_SET_OFFSET));
	}	
	
	
	if (read_flash(FLASH_WHEEL_BASE_SPEED_MODE_OFFSET) != -1) {
		wheel_base_set_speed_mode(read_flash(FLASH_WHEEL_BASE_SPEED_MODE_OFFSET));
	}
	
	if (read_flash(FLASH_ABS_ANGLE_MODE_OFFSET) != -1) {
		if (read_flash(FLASH_ABS_ANGLE_MODE_OFFSET) == 0 || read_flash(FLASH_ABS_ANGLE_MODE_OFFSET) == 1) {
			abs_angle_mode = read_flash(FLASH_ABS_ANGLE_MODE_OFFSET);
		}
	}
	
	if (read_flash(FLASH_ANGLE_LOCK_MODE_OFFSET) != -1) {
		if (read_flash(FLASH_ANGLE_LOCK_MODE_OFFSET) == 0 || read_flash(FLASH_ANGLE_LOCK_MODE_OFFSET) == 1) {
			angle_lock_mode = read_flash(FLASH_ANGLE_LOCK_MODE_OFFSET);
		}
	}
	
	
	
}

void robocon_main(void)
{

	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			// Every ms
			serving_update();
			upper_racket_update();
			angle_lock_update();
			
			if (ticks_img % 5 == 0) {
				wheel_base_pid_update();
				wheel_base_update();
				angle_variance_update();
			}
				
			if (ticks_img % 10 == 1) {
				// Every 10 ms (100 Hz)
				bluetooth_update();
			}
			
			if (ticks_img % 4 == 1) {
				angle_variance_update();
			}

			
			if (get_seconds() % 10 == 2 && ticks_img == 2) {
				// Every 10 seconds (0.1 Hz)
				battery_regular_check();
			}

			
			if (ticks_img % 200 == 3) {
				// Every 100 ms (10 Hz)
				//wheel_base_tx_position();
				// Sensor sound
				u32 sensor_val = get_sensor();
				switch (sensor_val / 200) {
					case 1:
						buzzer_control_note(20, 25, NOTE_D, 7);
					break;
					
					case 2:
					case 3:
					case 4:
					case 5:
					case 6:
						buzzer_control_note(10, 50, NOTE_D, 7);
					break;
					
					case 7:
					case 8:
					case 9:
						buzzer_control_note(5, 100, NOTE_D, 7);
					break;
				}
			}
			
			if (ticks_img % 500 == 4) {
				led_control(LED_D3, (LED_STATE) (ticks_img == 0));
			}
			
			if (ticks_img % 20 == 5) {		// 50Hz
				button_update();
				us_auto_update();
				
				
				
				if (angle_lock_mode) {
					if (angle_lock_ignored()) {
						gpio_write(GYRO_LED_GPIO, (BitAction) (get_ticks() % 250 < 125)); 
					} else {
						gpio_write(GYRO_LED_GPIO, (BitAction) 1);
					}
				} else if (!gyro_get_available()) {
					gpio_write(GYRO_LED_GPIO, (BitAction) (get_ticks() % 100 <= 20));
				} else {
					gpio_write(GYRO_LED_GPIO, (BitAction) 0);
				}
				if (!angle_lock_mode) {angle_lock_ignore(200); }
				
				if (serving_mode) {
					angle_lock_ignore(200);
				}
				
				robocon_get_xbc();
				
				// Auto-serve update
				if (auto_serve_mode) {
					u32 time_used = get_full_ticks() - auto_serve_time;
					
					if (time_used >= AUTO_SERVE_PRE_DELAY) {
						serving_hit_start();
						auto_serve_mode = false;
						serving_mode = false;
						auto_serve_time = 0;
						buzzer_control_note(1, 500, NOTE_D, 7);
					} else {
						u32 time_left = AUTO_SERVE_PRE_DELAY - time_used;
						if (time_left % 1000 < 50) {
							buzzer_control_note(1, 200, NOTE_D, 7); 
						}
						
						if (time_left < 3000 && time_left % 1000 >= 475 && time_left % 1000 <= 525) {
							buzzer_control_note(1, 200, NOTE_D, 6);
						}
					}
					
					
				}
				
				// LED indicator
				if (get_serving_cali_state() != SERVING_CALI_NULL) {
					gpio_write(SERVING_LED_GPIO, (BitAction) (ticks_img % 500 < 250));
				} else if (get_serving_hit_state() != SERVING_NULL) {
					gpio_write(SERVING_LED_GPIO, 1);
				} else if (get_serving_calibrated()) {
					gpio_write(SERVING_LED_GPIO, (BitAction) (ticks_img % 125 < 62));
				} else {
					gpio_write(SERVING_LED_GPIO, 0);
				}
				
				if (auto_serve_mode && get_full_ticks() - auto_serve_time < AUTO_SERVE_PRE_DELAY) {
					u32 time_left = AUTO_SERVE_PRE_DELAY - (get_full_ticks() - auto_serve_time);
					if (time_left >= 5000) {
						gpio_write(AUTO_SERVE_LED_GPIO, 1); 
					} else if (time_left >= 3000) {
						gpio_write(AUTO_SERVE_LED_GPIO, ticks_img % 500 < 250);
					} else {
						gpio_write(AUTO_SERVE_LED_GPIO, ticks_img % 250 < 125);
					}
				} else {
					gpio_write(AUTO_SERVE_LED_GPIO, 0);
				}
				
				if (us_auto_mode) {
					if (!serving_mode && get_serving_hit_state() == SERVING_NULL) {
						gpio_write(US_AUTO_LED_GPIO, 1);
						switch (us_auto_get_response()) {
						
							case US_AUTO_HIT: {
								u16 val = us_get_detection_val();
								upper_racket_hit(30);
								//buzzer_control_note(3, 50, NOTE_D, 7);
								gpio_write(SERVING_LED_GPIO, 0);
								break;
							}
								
							case US_AUTO_E_STOP_HIT: {
								upper_racket_e_stop();
								gpio_write(US_AUTO_LED_GPIO, (BitAction) (ticks_img % 100 < 50));
								
								break;
							}
							
							default:
							
							break;
						}
					} else {
						gpio_write(US_AUTO_LED_GPIO, (BitAction) (ticks_img % 200 < 100)); 
					}
				} else {
					gpio_write(US_AUTO_LED_GPIO, 0);
				}
				
				if (button_pressed(BUTTON_1) == 30 || button_pressed(BUTTON_2) == 30 || button_pressed(BUTTON_XBC_BACK) == 30) {
					/** Stop the wheel base before return **/
					wheel_base_stop();
					return; 
				}
				
				
			
			
			
				if (upper_racket_get_mode() == UPPER_RACKET_PRE_HIT) {
					gpio_write(UPPER_RACKET_GPIO, ticks_img % 200 < 100);
				} else if (upper_racket_get_mode() == UPPER_RACKET_HITTING) {
					gpio_write(UPPER_RACKET_GPIO, 1);
				} else if (upper_racket_get_mode() == UPPER_RACKET_POST_HIT) {
					gpio_write(UPPER_RACKET_GPIO, ticks_img % 100 < 50);
				} else if (serving_mode) {
					gpio_write(UPPER_RACKET_GPIO, ticks_img % 250 < 125);
				}	else {
					gpio_write(UPPER_RACKET_GPIO, 0); 
				}
			
			}
			
			if (ticks_img % 50 == 7) {
				// Every 50 ms (20 Hz)
				/** Warning: try not to do many things after tft_update(), as it takes time **/

				WHEEL_BASE_VEL vel_real = wheel_base_get_vel_real(), 
					vel_target = wheel_base_get_vel_target();
					
				tft_clear();
				draw_top_bar();

				tft_prints(0, 1, "(%3d,%3d,%3d)", vel_target.x, vel_target.y, vel_target.w);
				tft_prints(0, 2, "(%3d,%3d,%3d)", vel_real.x, vel_real.y, vel_real.w);
				tft_prints(0, 3, "Speed: %d", wheel_base_get_speed_mode());
				if (gyro_get_available()) {
					tft_prints(0, 4, "(%-4d,%-4d,%-4d)", get_pos()->x, get_pos()->y,get_pos()->angle);
				} else {
					tft_prints(0, 4, "[(%-4d,%-4d,%-4d)]", get_pos()->x, get_pos()->y, get_pos()->angle);
				}
				//tft_prints(0, 5, "%s", abs_angle_mode ? "[ABSOLUTE]" : "Relative");
				tft_prints(0, 5, "%d %d", gpio_read_input(NET_SENSOR_Q1), gpio_read_input(NET_SENSOR_Q2));
				tft_prints(0, 6, "State: (%d,%d)", get_serving_cali_state(), get_serving_hit_state());
				
				//tft_prints(0, 6, get_serving_calibrated() ? "CALI" : "[NOT CAL!]"); 
				//tft_prints(10, 6, get_serving_switch() ? "SW": "--");
				s32 target_encoder = 0;
				if (get_serving_cali_state() != SERVING_CALI_NULL) {
					target_encoder = get_serving_cali_encoder_target();
				} else if (get_serving_hit_state() != SERVING_NULL) {
					target_encoder = get_serving_hit_encoder_target();
				}
				tft_prints(0, 7, "%d->%d", get_serving_encoder(), target_encoder);
				
				//tft_prints(0, 7, "Serve:%d,%d", get_shuttle_drop_delay(), get_serving_hit_speed());
				tft_prints(0, 8, "S%d:%d,%d", get_serving_set(), get_shuttle_drop_delay(), get_serving_hit_speed());
				
				tft_prints(0, 9, "AV:%d", get_angle_variance());
				
				tft_prints(6, 9, "S:%d", get_sensor());
				/*
				for (u8 i = 0; i < US_AUTO_DEVICE_COUNT; ++i) {
					u16 dist = us_get_distance(i);
					if (dist == 0) {
						tft_prints(i, 8, "+");
					} else if (dist < 1000) {
						tft_prints(i, 8, "%d", (dist / 100) % 10);
					} else {
						tft_prints(i, 8, "+");
					}
				}
				
				tft_prints(11, 8, "%d", angle_lock_get_mv());
				tft_prints(11, 9, "%d", angle_lock_get_target());
				US_AUTO_RESPONSE us_response = us_auto_get_response();
				
				if (us_response == US_AUTO_NULL) { 
					tft_prints(0, 9, "---");
				} else if (us_response == US_AUTO_HIT) {
					tft_prints(0, 9, "HIT");
				} else if (us_response == US_AUTO_HITTING) {
					tft_prints(0, 9, "...");
				}	else if (us_response == US_AUTO_E_STOP_HIT) {
					tft_prints(0, 9, "[XXX]");
				}
				
				tft_prints(5, 9, "%d", us_get_detection_val());
				*/
				
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
