#include "robocon.h"
// relative movement: always move to front regardless the angle
#define absolute_angle

//#include "log.h"

//system values
static u16 ticks_img 	= (u16)-1;
static u32 last_sent_OS_time = 0;
static bool key_trigger_enable = true;
//static u16 prev_ticks=-1;
// Omega PID relevant.
static int accumulated_omega = 0;
static int target_angle = 0;

static u16 tick_skip_count=0;

// Enable flag.
bool serve_pneu_button_enabled=1;
bool turn_timer_started = 0;
static bool force_terminate = false;

// enabling sensors by button.
bool sensors_activated = 0;
bool emergency_serve_button_pressed=0;

// emergency serve activated
bool emergency_serve_activated = 0;
u32 emergency_serve_start_time=0;
bool remove_robot_sequence_started=0;
u32 remove_robot_start_time=0;
bool emergency_serve_hitting=0;

s32 angle = 0;
// bound const
const u16 MAX_ACCEL_BOOST = 2236;
const u16 MIN_ACCEL_BOOST = 707;
const u16 MAX_DELAY = 1000;
const u16 MIN_DELAY = 50;
const u16 MAX_VEL = 1800;
const u16 MIN_VEL = 800;

const u32 MAX_TIMEOUT	= 150;
// default const
const u16 DEFAULT_ACCEL_BOOST = 1414;
/*
const u16 DEFAULT_VEL[SERVE_SET_COUNT] = {1300};
const u16 DEFAULT_DELAY[SERVE_SET_COUNT] = {270};
*/

const u16 ACCELBOOSTER_OFFSET = 0;
const u16 SERVE_VEL_OFFSET[SERVE_SET_COUNT] = {1, 2};
const u16 SERVE_DELAY_OFFSET[SERVE_SET_COUNT] = {3, 4};


bool is_force_terminate(void)
{
	return force_terminate;
}

bool robot_xbc_controls(void)
{
	
	//Update Button Data
	button_update();
	/*
	* Left Analog - Move(Analog)
	* Y + Gamepad Buttons - serve delay and motor speed tunning
	* Gamepad (Left and right) - acceleration factor.
	* XBOX - force stop the Robot, for emergency usage
	* start - force stop for Robot C, which cannot use XBOX button
	* LT - CCW Turning(Analog)
	* RT - CW Turning(Analog)
	* A - Racket Hit
	* B - Serve
	* X - Calibrate
	* RB - Reset Gyro
	* LB + B - Fake serve
	* 
	*/
	
	//Analog Movement
	//Set x and y vel according to analog stick input
	int raw_vx = xbc_get_joy(XBC_JOY_LX);
	int raw_vy = xbc_get_joy(XBC_JOY_LY);
	
	int h = Sqrt(Sqr(raw_vx)+ Sqr(raw_vy));
	
	// Scalar Speed limit
	if (h > XBC_JOY_SCALE) {
		raw_vx = raw_vx*XBC_JOY_SCALE / h;
		raw_vy = raw_vy*XBC_JOY_SCALE / h;
	}
	
	// Set output x and y.
	s32 vx = raw_vx;
	s32 vy = raw_vy;
	
	//Use rotation matrix to control the robots by absolute coordinates not relative
	#ifdef absolute_angle
	int current_angle = get_pos()->angle;
	// All sin and cos are multiplied by 10000, so we divide it for output.
	vx = (- raw_vy * int_sin(current_angle) + raw_vx * int_cos(current_angle)) / 10000;
	vy = (raw_vy * int_cos(current_angle) + raw_vx * int_sin(current_angle)) / 10000;
	#endif
	
	// Violation prevention
	#if (ROBOT == 'C') 
		const int allowed_speed = XBC_JOY_SCALE / 3;
		if (vy > 0 && is_force_stop() && is_force_decel()) {
			// Force stop
			vy = 0;
      FAIL_MUSIC;
		}	else if (vy > allowed_speed && is_force_decel()) {
			// Force decel
			vy = vy * allowed_speed / XBC_JOY_SCALE;
      CLICK_MUSIC;
		}
	#endif
	
	// Spining velocity
  int omega = (xbc_get_joy(XBC_JOY_RT)-xbc_get_joy(XBC_JOY_LT))/5;
	const int speed_factor = 
	#if (ROBOT == 'C') 
		6
	#else 
		10
	#endif
	;

	// Accumulated the floating value
  accumulated_omega += omega % speed_factor;
  int incremental = accumulated_omega / speed_factor;
  if (incremental != 0) {
    target_angle += incremental;
    accumulated_omega -= incremental * speed_factor; 
  }
	
	// changing target angle with int value.
	target_angle += omega / speed_factor;
	if (target_angle < 0) {
		target_angle += 3600;
	} else if (target_angle > 3599) {
		target_angle -= 3600;
	}
	
	wheel_base_set_target_pos((POSITION){get_pos()->x, get_pos()->y, target_angle});

	// Get angular speed based on target_angle
	int vw = pid_maintain_angle();
	
	if (!emergency_serve_activated && !force_terminate)
		wheel_base_set_vel(vx, vy, vw);
	if (emergency_serve_activated && (raw_vx!=0 || raw_vy!=0 || omega!=0))	// cancel emergency serve
	{
		//SUCCESSFUL_SOUND;
		if (!emergency_serve_hitting)
		{
			serve_free();
		}
		emergency_serve_activated=0;
	}
	
	robot_cd_common_function();
	#if (ROBOT=='C')
	 	robot_c_function_controls();
	#elif (ROBOT=='D')
		robot_d_function_controls();
	#endif
	
	return true;
}


static void robot_cd_common_function(void)
{
	if (button_pressed(BUTTON_XBC_XBOX)) {
		// forced terminate everything when pressed.
		// Stop spinning
		target_angle = get_pos()->angle;
		accumulated_omega = 0;
		// Stop Motor.
		force_terminate = true;
		wheel_base_set_vel(0, 0, 0);
		// Serving part reset.
		serve_free();
		// Ignore other button.
    return;
  } else if (button_released(BUTTON_XBC_XBOX)) {
		force_terminate = false;
	}
	
	if (button_pressed(BUTTON_XBC_RB)) {
		gyro_pos_set(get_pos()->x, get_pos()->y, 0);
		accumulated_omega = target_angle = 0;
		wheel_base_tx_acc();
	}	
}

#if (ROBOT == 'C')
void robot_c_function_controls(void)
{
	/* Sensor is not in use now
	// enable sensors
	if(button_pressed(BUTTON_XBC_RB))
		sensors_activated=1;
	else
		sensors_activated=0;
	*/
	if (button_pressed(BUTTON_XBC_START) == 3) {
		force_terminate = !force_terminate;
  }
	
	if (force_terminate) {
		// forced terminate everything when pressed.
		// Stop spinning
		target_angle = get_pos()->angle;
		accumulated_omega = 0;
		// Stop Motor.
		wheel_base_set_vel(0, 0, 0);
		// Serving part reset.
		serve_free();
		// Ignore other button.
		FAIL_MUSIC;
    return;
	}
	
	if (button_pressed(BUTTON_XBC_LB) == 3) {
		CLICK_MUSIC;
		gyro_cal();
	}
	
	if ((button_pressed(BUTTON_XBC_E) == 1 || button_pressed(BUTTON_XBC_E) >= 50) && accel_booster < MAX_ACCEL_BOOST) {
		++accel_booster;
	} else if ((button_pressed(BUTTON_XBC_W) == 1|| button_pressed(BUTTON_XBC_W) >= 50) && accel_booster > MIN_ACCEL_BOOST){
		--accel_booster;
	} else if (button_released(BUTTON_XBC_E) == 1 || button_released(BUTTON_XBC_W) == 1) {
		write_flash(ACCELBOOSTER_OFFSET, accel_booster);
	}
	
	//Racket Hit
	if(button_pressed(BUTTON_XBC_LB) && button_pressed(BUTTON_XBC_B))
		racket_hit(70);
	else if(button_pressed(BUTTON_XBC_B))
		racket_hit(500);
	if(button_pressed(BUTTON_XBC_A))
		racket_down_hit();
	//if(button_pressed(BUTTON_XBC_RB))
	//	plus_x();
	//if(button_pressed(BUTTON_XBC_LB))
	//	minus_x();
	//if(button_pressed(BUTTON_XBC_X))
	//	plus_y();
	//if(button_pressed(BUTTON_XBC_Y))
	//	minus_y();
}
#endif

#if (ROBOT == 'D') 
void robot_d_function_controls(void)
{
	//Racket Hit
	if(button_pressed(BUTTON_XBC_LB) && button_pressed(BUTTON_XBC_A))
		racket_hit(70);
	else if(button_pressed(BUTTON_XBC_A))
		racket_hit(500);
	//Calibrate
	else if (button_pressed(BUTTON_XBC_LB) && button_pressed(BUTTON_XBC_B))
		serve_start(1);
	else if(button_pressed(BUTTON_XBC_B))
		serve_start(0);
	else if(button_pressed(BUTTON_XBC_X))//Not essential
		serve_calibrate();
		
	if (button_pressed(BUTTON_XBC_Y))  {
		// serve varibales 1
		if (!button_pressed(BUTTON_XBC_LB)) { 
			if (button_pressed(BUTTON_XBC_N) && serve_get_delay(0) < MAX_DELAY)
				serve_change_delay(0, 1);
			else if (button_pressed(BUTTON_XBC_S) && serve_get_delay(0) > MIN_DELAY)
				serve_change_delay(0, -1);
			else if (button_pressed(BUTTON_XBC_W) && serve_get_vel(0) > MIN_VEL)
				serve_change_vel(0, -2);
				//plus_x();
			else if (button_pressed(BUTTON_XBC_E) && serve_get_vel(0) < MAX_VEL)
				serve_change_vel(0, 2);
		}
		
		// serve variable 2
		if (button_pressed(BUTTON_XBC_LB)) { 
			if (button_pressed(BUTTON_XBC_N) && serve_get_delay(1) < MAX_DELAY)
				serve_change_delay(1, 1);
			else if (button_pressed(BUTTON_XBC_S) && serve_get_delay(1) > MIN_DELAY)
				serve_change_delay(1, -1);
			else if (button_pressed(BUTTON_XBC_W) && serve_get_vel(1) > MIN_VEL)
				serve_change_vel(1, -2);
				//plus_x();
			else if (button_pressed(BUTTON_XBC_E) && serve_get_vel(1) < MAX_VEL)
				serve_change_vel(1, 2);
		}
	
	}
	
		//minus_x();
	// acceleration rate tunning.
	else if ((button_pressed(BUTTON_XBC_E) == 1 || button_pressed(BUTTON_XBC_E) >= 50) && accel_booster < MAX_ACCEL_BOOST) {
		++accel_booster;
	} else if ((button_pressed(BUTTON_XBC_W) == 1 || button_pressed(BUTTON_XBC_W) >= 50) && accel_booster > MIN_ACCEL_BOOST){
		--accel_booster;
	}
	
	if (button_released(BUTTON_XBC_E) == 1 || (button_released(BUTTON_XBC_W)) == 1) {
		write_flash(ACCELBOOSTER_OFFSET, accel_booster);
		write_flash(SERVE_VEL_OFFSET[0], serve_get_vel(0));
		write_flash(SERVE_VEL_OFFSET[1], serve_get_vel(1));
	} else if (button_released(BUTTON_XBC_N) == 1 || (button_released(BUTTON_XBC_S)) == 1) {
		write_flash(SERVE_DELAY_OFFSET[0], serve_get_delay(0));
		write_flash(SERVE_DELAY_OFFSET[1], serve_get_delay(1));
	}
	
	if (button_pressed(BUTTON_XBC_START))
	{
		serve_free();
	}
	
	/*manual pneumatic trigger*/
	if (serve_pneu_button_enabled && gpio_read_input(SERVE_PNEU_TEST))
	{
		serve_pneu_button_enabled=0;
		serve_pneu_toggle();
	}
	if (!gpio_read_input(SERVE_PNEU_TEST))
	{
		serve_pneu_button_enabled=1;
	}
	
	/*emergency auto serve*/
	if (!emergency_serve_activated && gpio_read_input(E_STOP_BUTTON))//on press
	{
		emergency_serve_button_pressed=1;
		emergency_serve_activated=0;
	}
	if (emergency_serve_button_pressed && !gpio_read_input(E_STOP_BUTTON))//on release
	{
		emergency_serve_start_time=get_full_ticks();
		emergency_serve_button_pressed=0;
		emergency_serve_activated=1;
		log("emergency",0);
		serve_calibrate();
		emergency_serve_hitting=0;
	}
	if (emergency_serve_activated)
	{
		const int AUTO_SERVE_DELAY = 10000;	// 10 seconds
		if (emergency_serve_start_time+AUTO_SERVE_DELAY<get_full_ticks() && emergency_serve_start_time+AUTO_SERVE_DELAY+300>get_full_ticks())//autoserve after 6 secs
		{
			start_auto_serve();
			emergency_serve_hitting=1;
		}
		if (emergency_serve_start_time+(1000 + AUTO_SERVE_DELAY)<get_full_ticks())	// remove robot after serve
		{
			log("remove",get_full_ticks());
			//SUCCESSFUL_SOUND;
			remove_robot_sequence_started=1;
			wheel_base_set_vel(100,0,0);
			emergency_serve_hitting=0;
		}
		if (emergency_serve_start_time+(AUTO_SERVE_DELAY+2100)<get_full_ticks())//remove robot complete
		{
			serve_free();
			log("removed",get_full_ticks());
			buzzer_play_song(BIRTHDAY_SONG, 120, 0);
			remove_robot_sequence_started=0;
			emergency_serve_activated=0;
			wheel_base_set_vel(0,0,0);
			wheel_base_update();
			
		}
	}
	/**
	* NEC controls
	
	*/
	if (nec_get_msg(0)->address==0x40 && nec_get_msg(0)->command==0x05)
	{
		racket_hit(500);
	}
}
#endif

static void handle_bluetooth_input(void)
{
	robot_xbc_controls();
	if (/*!robot_xbc_controls() && */key_trigger_enable && !bluetooth_is_key_release())
	{
		//set_xbc_input_allowed(false);
		last_sent_OS_time = get_full_ticks();
		key_trigger_enable = false;
		//wheel_base_pid_off();
		switch (wheel_base_bluetooth_get_last_char())
		{
			case 'k':
				racket_hit(500);
			break;
			case 'j':
				#if (ROBOT=='C')
					racket_down_hit();
				#endif
			break;
			case 'l':
				#if (ROBOT == 'D')
					serve_calibrate();
				#endif
			break;
			case 'o':
				#if (ROBOT == 'D')
					motor_lock(RACKET);
				#endif
			break;
			case 'y':
				#if (ROBOT == 'D')
				//is_laser_serve_enabled(1);
					serve_pneu_toggle();
				#endif
				break;
			case 'u'://Normal Serve
				#if (ROBOT == 'D')
				//is_laser_serve_enabled(0);
					serve_start(0);
				#endif
				
				break;
			
			case 'p':
				#if (ROBOT == 'D')
					serve_free();
					key_trigger_enable = true;
				#endif
			break;
			case '=':
				//For both robots
				//serve_change_vel(2);
				key_trigger_enable = true;
				break;
			case '-':
				//serve_change_vel(-2);
				key_trigger_enable = true;
				break;
			case '.':
				//serve_change_delay(+5);
				break;
			case ',':
				//serve_change_delay(-5);
				break;
			case '[':		//only temporary
				//plus_x();
				key_trigger_enable = true;
				break;
			case ']':		// only temporary
				//minus_x();
				key_trigger_enable = true;
				break;
			case '{':		//only temporary
				//plus_y();
				key_trigger_enable = true;
				break;
			case '}':		// only temporary
				//minus_y();
				key_trigger_enable = true;
				break;
			
			//Increase Speed Mode
			case 'P':
				wheel_base_set_speed_mode(wheel_base_get_speed_mode() + 1);
				break;
			//Decrease Speed Mode
			case 'O':
				wheel_base_set_speed_mode(wheel_base_get_speed_mode() - 1);
				break;
			
			case 'f':
			case 'g':
				wheel_base_pid_off();
	 }
	}
	else if (bluetooth_is_key_release())
	{
		key_trigger_enable = true;
	}
	
}







void robocon_main(void)
{
	motor_set_acceleration(RACKET, 5000);
  // Send the acceleration data
	wheel_base_tx_acc();
	serve_free();
	
	#if (ROBOT == 'D')
	serve_timer_init();
	gpio_init(SERVE_PNEU0_GPIO,  GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);		// Serve pneumatric GPIO Robot D, GEN2
	gpio_init(SERVE_PNEU1_GPIO,  GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);		// Serve pneumatric GPIO Robot D, GEN2. MOSFET burnt
	gpio_init(SERVE_PNEU0_GPIO_BACKUP,  GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);	
	gpio_init(SERVE_PNEU1_GPIO_BACKUP,  GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);	
	
	
	
	
	gpio_init(SERVE_SWITCH, GPIO_Speed_10MHz, GPIO_Mode_IPU, 1);	// Mechanical switch ROBOT D Gen2
	gpio_init(SERVE_PNEU_TEST, GPIO_Speed_10MHz, GPIO_Mode_IPD, 1);	// Shuttlecock Holder button for ROBOT D Gen2
	gpio_init(E_STOP_BUTTON, GPIO_Speed_10MHz, GPIO_Mode_IPD, 1);	// Emergency serve button for ROBOT D Gen2
	#endif

	gpio_init(PNEU_GPIO, GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);			// pneu matic GPIO
	gpio_init(PNEU_GPIO_DOWN, GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);
	
	
	gpio_init(&PA4,GPIO_Speed_50MHz, GPIO_Mode_IPD,1);		// laser sensor
	gpio_init(&PA6,GPIO_Speed_50MHz, GPIO_Mode_IPD,1);	// laser sensor grid 2
	gpio_init(&PA7,GPIO_Speed_50MHz, GPIO_Mode_IPD,1);	// laser sensor grid 3
	
	serve_pneu_set(0, false);
	serve_pneu_set(1, false);
	racket_pneumatic_set(0);
	racket_pneumatic_2_set(0);
	
	wheel_base_set_target_pos((POSITION){get_pos()->x, get_pos()->y, 0}); 
	
	log("XBC=",xbc_get_connection());
	// Read from flash
	accel_booster = read_flash(ACCELBOOSTER_OFFSET);
	
	#if (ROBOT == 'D') 
		u32 vel[] = {read_flash(SERVE_VEL_OFFSET[0]), read_flash(SERVE_VEL_OFFSET[1])};
		u32 delay[] = {read_flash(SERVE_DELAY_OFFSET[0]), read_flash(SERVE_DELAY_OFFSET[1])};
		
		if (vel[0] >= MIN_VEL && vel[0] <= MAX_VEL) {
			serve_set_vel(0, vel[0]);
		}
		
		if (delay[0] >= MIN_DELAY && delay[0] <= MAX_DELAY) {
			serve_set_delay(0, delay[0]);
		}
		
		if (vel[1] >= MIN_VEL && vel[1] <= MAX_VEL) {
			serve_set_vel(1, vel[1]);
		}
		
		if (delay[1] >= MIN_DELAY && delay[1] <= MAX_DELAY) {
			serve_set_delay(1, delay[1]); 
		}
		
		
	#endif
	
	// if no memory, set to default value and save it
	if (accel_booster < MIN_ACCEL_BOOST || accel_booster > MAX_ACCEL_BOOST) {
		accel_booster = DEFAULT_ACCEL_BOOST;
		write_flash(ACCELBOOSTER_OFFSET, accel_booster);
	}
	
	/*
	#if (ROBOT == 'D') 
	// Protection
		if (serve_get_vel(0) < MIN_VEL || serve_get_vel(0) > MAX_VEL) {
			serve_set_vel(0, DEFAULT_VEL);
			write_flash(SERVE_VEL_OFFSET[0], serve_get_vel(0)); 
		}
		
		if (serve_get_delay(0) < MIN_DELAY || serve_get_delay(0) > MAX_DELAY) {
			serve_set_delay(0, DEFAULT_DELAY);
			write_flash(SERVE_DELAY_OFFSET[0], serve_get_delay(0));
		}
		
		if (serve_get_vel(1) < MIN_VEL || serve_get_vel(1) > MAX_VEL) {
			serve_set_vel(1, DEFAULT_VEL);
			write_flash(SERVE_VEL_OFFSET[1], serve_get_vel(1)); 
		}
		
		if (serve_get_delay(1) < MIN_DELAY || serve_get_delay(1) > MAX_DELAY) {
			serve_set_delay(1, DEFAULT_DELAY);
			write_flash(SERVE_DELAY_OFFSET[1], serve_get_delay(1));
		}
	#endif
	*/
	
	while (1) {
		if (ticks_img != get_ticks()) {
//			if (prev_ticks!=ticks_img-1)
//				tick_skip_count+=1;//(ticks_img-1-prev_ticks);
//			prev_ticks=ticks_img;
			
			ticks_img = get_ticks();
			//if (ticks_img%2 == 1)
			//{
//				if (sensors_activated && !serve_prioritized())
//					sensors_update();			// only update sensors when serve is not prioritized.
				//racket_update();
			//}
			if (ticks_img % 10 == 0) {
        //wheel_base_update();	//wheel_base_update now also handles auto positioning system
				bluetooth_update();
        handle_bluetooth_input();
 
        button_update();
				// Every 10 ms (100 Hz)
         if (return_listener()) {
          return;
        }
			}
			// wheel_base update every milisecond
			wheel_base_update();
			
				
			/*if (ticks_img % 250 == 1) {
				// Every 250 ms (4 Hz)
				//battery_adc_update();
				//send_string_s("hello\n", 6);
				send_string("hi");
			}*/
			
			
//			if (get_seconds() % 10 == 2 && ticks_img == 2) {
//				// Every 10 seconds (0.1 Hz)
//				battery_regular_check();
//			}

			
//			if (ticks_img % 100 == 3 && !serve_prioritized()) {
//				// Every 100 ms (10 Hz)
//				wheel_base_tx_position();
//			}
//			
//			if (ticks_img % 500 == 4 && !serve_prioritized()) {
//				led_control(LED_D3, (LED_STATE) (ticks_img == 0));
//			}
			
			if (ticks_img % 50 == 5 && !serve_prioritized()) {
				//wheel_base_joystick_control();
				if (button_pressed(BUTTON_1) > 10 || button_pressed(BUTTON_2) > 10 || button_pressed(BUTTON_XBC_BACK) > 10) {
					/** Stop the wheel base before return **/
					return; 
				}
			}
			
			
			if (!serve_prioritized() && ticks_img % 50 == 7) {
				// Every 50 ms (20 Hz)
				/** Warning: try not to do many things after tft_update(), as it takes time **/

				WHEEL_BASE_VEL vel = wheel_base_get_vel();
				tft_clear();

				tft_prints(0, 1, "V:(%3d,%3d,%3d)", vel.x, vel.y, vel.w);
				//tft_prints(0, 2, "Speed: %d", wheel_base_get_speed_mode());
				#if (ROBOT == 'D')
					if (serve_get_failed()) {
						tft_prints(0,2,"[Encoder:--]");
					} else {
						tft_prints(0,2,"Encoder:%d", get_encoder_value(RACKET));
					}
					
					tft_prints(0,3,"S0:%3d %3d", serve_get_delay(0), serve_get_vel(0));
					tft_prints(0,4,"S1:%3d %3d", serve_get_delay(1), serve_get_vel(1));
					tft_prints(0,5,"Timeout:%3d", serve_get_timeout());
				#else
          tft_prints(0,2,"Decel: %d", is_force_decel());
          tft_prints(0,3,"Stop: %d", is_force_stop());
        #endif
				
				tft_prints(0,6,"Target: %d", wheel_base_get_target_pos().angle);
				tft_prints(0,7,"Angle: %d", get_pos()->angle);
				tft_prints(0,8,"accel rate: %d", accel_booster);
//				tft_prints(0,7,"skipTick: %d",tick_skip_count);
				
				if (force_terminate) {
					tft_prints(0,9, "STOP!");
				}
					 
        //log_update();
				tft_update();
			}
			
		}
		
	}	
}
