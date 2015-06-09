#include "robocon.h"
// relative movement: always move to front regardless the angle
#define absolute_angle

//#include "log.h"

//system values
static u16 ticks_img 	= (u16)-1;
static u32 last_sent_OS_time = 0;
static bool key_trigger_enable = true;
static u16 prev_ticks=-1;
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
	
	// Spining velocity
  int omega = (xbc_get_joy(XBC_JOY_RT)-xbc_get_joy(XBC_JOY_LT))/5;
	const int speed_factor = ROBOT=='C'?6:10;

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
	if (ROBOT=='C')
	 	robot_c_function_controls();
	if (ROBOT=='D')
		robot_d_function_controls();
	
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
	}	
}

void robot_c_function_controls(void)
{
	/* Sensor is not in use now
	// enable sensors
	if(button_pressed(BUTTON_XBC_RB))
		sensors_activated=1;
	else
		sensors_activated=0;
	*/
	
	if (button_pressed(BUTTON_XBC_E) && accel_booster < 1732) {
		++accel_booster;
	} else if (button_pressed(BUTTON_XBC_W) && accel_booster > 707){
		--accel_booster;
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

void robot_d_function_controls(void)
{
	//Racket Hit
	if(button_pressed(BUTTON_XBC_LB) && button_pressed(BUTTON_XBC_A))
		racket_hit(70);
	else if(button_pressed(BUTTON_XBC_A))
		racket_hit(500);
	//Calibrate
	else if (button_pressed(BUTTON_XBC_LB) && button_pressed(BUTTON_XBC_B))
		fake_serve_start();
	else if(button_pressed(BUTTON_XBC_B))
		serve_start();
	else if(button_pressed(BUTTON_XBC_X))//Not essential
		serve_calibrate();
	// change serve varibales
	
	if (button_pressed(BUTTON_XBC_N) && button_pressed(BUTTON_XBC_Y))
		serve_change_delay(1);
	else if (button_pressed(BUTTON_XBC_S) && button_pressed(BUTTON_XBC_Y))
		serve_change_delay(-1);
	else if (button_pressed(BUTTON_XBC_W) && button_pressed(BUTTON_XBC_Y))
		serve_change_vel(-2);
		//plus_x();
	else if (button_pressed(BUTTON_XBC_E) && button_pressed(BUTTON_XBC_Y))
		serve_change_vel(2);
		//minus_x();
	// acceleration rate tunning.
	else if (button_pressed(BUTTON_XBC_E) && accel_booster < 1732) {
		++accel_booster;
	} else if (button_pressed(BUTTON_XBC_W) && accel_booster > 707){
		--accel_booster;
	}
	
	if (button_pressed(BUTTON_XBC_START))
	{
		serve_free();
	}
	
	/*manual pneumatic trigger*/
	if (serve_pneu_button_enabled && gpio_read_input(&PE5))
	{
		serve_pneu_button_enabled=0;
		toggle_serve_pneu();
	}
	if (!gpio_read_input(&PE5))
	{
		serve_pneu_button_enabled=1;
	}
	
	/*emergency auto serve*/
	if (!emergency_serve_activated && gpio_read_input(&PE3))//on press
	{
		emergency_serve_button_pressed=1;
		emergency_serve_activated=0;
	}
	if (emergency_serve_button_pressed && !gpio_read_input(&PE3))//on release
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
		if (emergency_serve_start_time+6000<get_full_ticks() && emergency_serve_start_time+6300>get_full_ticks())//autoserve after 6 secs
		{
			start_auto_serve();
			emergency_serve_hitting=1;
		}
		if (emergency_serve_start_time+7000<get_full_ticks())	// remove robot after serve
		{
			log("remove",get_full_ticks());
			//SUCCESSFUL_SOUND;
			remove_robot_sequence_started=1;
			wheel_base_set_vel(100,0,0);
			emergency_serve_hitting=0;
		}
		if (emergency_serve_start_time+8100<get_full_ticks())//remove robot complete
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
				if (ROBOT=='C')
					racket_down_hit();
			break;
			case 'l':
				if(ROBOT == 'D')
					serve_calibrate();
			break;
			case 'o':
				if(ROBOT == 'D')
					motor_lock(RACKET);
			break;
			case 'y':
				if(ROBOT == 'D'){
				//is_laser_serve_enabled(1);
					toggle_serve_pneu();
				}
				break;
			case 'u'://Normal Serve
				if(ROBOT == 'D'){
				//is_laser_serve_enabled(0);
					serve_start();
				}
				
				break;
			
			case 'p':
				if(ROBOT == 'D')
				serve_free();
				key_trigger_enable = true;
			break;
			case '=':
				//For both robots
				serve_change_vel(2);
				key_trigger_enable = true;
				break;
			case '-':
				serve_change_vel(-2);
				key_trigger_enable = true;
				break;
			case '.':
				serve_change_delay(+5);
				break;
			case ',':
				serve_change_delay(-5);
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
	gpio_init(&PD9,  GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);		// Serve pneumatric GPIO Robot D, GEN2
	gpio_init(&PE15,  GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);		// Serve pneumatric GPIO Robot D, GEN2. MOSFET burnt
	gpio_init(&PD10, GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);		// pneu matic GPIO
	
	
	gpio_init(&PE11, GPIO_Speed_10MHz, GPIO_Mode_IPU, 1);	// Mechanical switch ROBOT D Gen2
	gpio_init(&PE5, GPIO_Speed_10MHz, GPIO_Mode_IPU, 1);	// Shuttlecock Holder button for ROBOT D Gen2
	gpio_init(&PE3, GPIO_Speed_10MHz, GPIO_Mode_IPU, 1);	// Emergency serve button for ROBOT D Gen2

	gpio_init(&PA4,GPIO_Speed_50MHz, GPIO_Mode_IPD,1);		// laser sensor
	gpio_init(&PA6,GPIO_Speed_50MHz, GPIO_Mode_IPD,1);	// laser sensor grid 2
	gpio_init(&PA7,GPIO_Speed_50MHz, GPIO_Mode_IPD,1);	// laser sensor grid 3
	
	toggle_serve_pneu();
	racket_pneumatic_set(0);
	racket_pneumatic_2_set(0);
	
	wheel_base_set_target_pos((POSITION){get_pos()->x, get_pos()->y, 0}); 
	
	log("XBC=",xbc_get_connection());
	
	while (1) {
		if (ticks_img != get_ticks()) {
			if (prev_ticks!=ticks_img-1)
				tick_skip_count+=1;//(ticks_img-1-prev_ticks);
			prev_ticks=ticks_img;
			
			ticks_img = get_ticks();
			//if (ticks_img%2 == 1)
			//{
				if (sensors_activated && !serve_prioritized())
					sensors_update();			// only update sensors when serve is not prioritized.
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
			
			
			if (get_seconds() % 10 == 2 && ticks_img == 2) {
				// Every 10 seconds (0.1 Hz)
				battery_regular_check();
			}

			
			if (ticks_img % 100 == 3 && !serve_prioritized()) {
				// Every 100 ms (10 Hz)
				wheel_base_tx_position();
			}
			
			if (ticks_img % 500 == 4 && !serve_prioritized()) {
				led_control(LED_D3, (LED_STATE) (ticks_img == 0));
			}
			
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
				draw_top_bar();

				tft_prints(0, 1, "V:(%3d,%3d,%3d)", vel.x, vel.y, vel.w);
				//tft_prints(0, 2, "Speed: %d", wheel_base_get_speed_mode());
				char s[3] = {wheel_base_bluetooth_get_last_char(), '\0'};
        if (s[0] == '[' || s[0] == ']') {
          // Replace "[" and "]" as "\[" and "\]"
          s[1] = s[0];
          s[0] = '\\';
        }
				
				
				
				u8 connect = xbc_get_connection() == XBC_DISCONNECTED ? 0 : 1;
				
				if (ROBOT == 'D') {
					tft_prints(0,2,"Encoder: %d", get_encoder_value(RACKET));
					tft_prints(0,3,"Serve_delay: %d",serve_get_delay());
					tft_prints(0,4,"Racket: %d", serve_get_vel());
				}
				
				tft_prints(0,5,"Target: %d", wheel_base_get_target_pos().angle);
				tft_prints(0,6,"Angle: %d", get_pos()->angle);
				//tft_prints(0,7,"accel rate: %d", accel_booster);
				tft_prints(0,7,"skipTick: %d",tick_skip_count);
				
				if (force_terminate) {
					tft_prints(0,8, "STOP!");
				}
					 
        //log_update();
				tft_update();
			}
			
		}
		
	}	
}
