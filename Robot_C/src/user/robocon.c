#include "robocon.h"
// relative movement: always move to front regardless the angle
#define absolute_angle

//#include "log.h"

//system values
static u16 ticks_img 	= (u16)-1;
static u32 last_sent_OS_time = 0;
static bool key_trigger_enable = true;
static bool servo_released = false;
static bool use_xbc_input = false;
static bool force_terminate = false;
static u16 prev_ticks=-1;
static u16 turn_timer=-1;
static int accumulated_omega = 0;
static int target_angle = 0;

static u16 tick_skip_count=0;

bool serve_pneu_button_enabled=1;
bool turn_timer_started = 0;

// enabling sensors by button.
bool sensors_activated = 0;

u16 prev_vels[50] = { 0 };
u16 vel_index = 0;
u16 acc_mod = 0;

s32 angle = 0;

bool robot_xbc_controls(void)
{
	//Update Button Data
	button_update();
	/*
	* Left Analog - Move(Analog)
	* Gamepad Buttons - Move(Digital) - Speed depends on speed_mode
	* LT - CCW Turning(Analog)
	* RT - CW Turning(Analog)
	* A - Racket Hit
	* RB - Increase Movement Speed
	* LB - Decrease Movement Speed
	* 
	*/
	//Analog Movement
	
	
	//int dw = (xbc_get_joy(XBC_JOY_RT)-xbc_get_joy(XBC_JOY_LT))/5;

	//Set x and y vel according to analog stick input
	
	int vx = xbc_get_joy(XBC_JOY_LX);
	int vy = xbc_get_joy(XBC_JOY_LY);
	
	int h = Sqrt(vx*vx + vy*vy);
//	int cosine = vx / Sqrt(vx*vx + vy*vy);
//	int sine = vy / Sqrt(vx*vx + vy*vy);
	
	// Scalar Speed limit
	if (h > XBC_JOY_SCALE) {
		vx = vx*XBC_JOY_SCALE / h;
		vy = vy*XBC_JOY_SCALE / h;
	}
	s32 dx = vx;
	s32 dy = vy;
	//Use rotation matrix to control the robots by absolute coordinates not relative
	#ifdef absolute_angle
	int current_angle = get_pos()->angle;
	dy = (vy * int_cos(current_angle) + vx * int_sin(current_angle)) / 10000;
	dx = (- vy * int_sin(current_angle) + vx * int_cos(current_angle)) / 10000;
	#endif
	
	// Spining velocity
  int omega = (xbc_get_joy(XBC_JOY_RT)-xbc_get_joy(XBC_JOY_LT))/5;
	const int speed_factor = ROBOT=='C'?6:10;


  accumulated_omega += omega % speed_factor;
  int incremental = accumulated_omega / speed_factor;
  if (incremental != 0) {
    target_angle += incremental;
    accumulated_omega -= incremental * speed_factor; 
  }
	target_angle += omega / speed_factor;
	if (target_angle < 0) {
		target_angle += 3600;
	} else if (target_angle > 3599) {
		target_angle -= 3600;
	}
	
	wheel_base_set_target_pos((POSITION){get_pos()->x, get_pos()->y, target_angle});

	//Get angle speed based on target_angle
	int dw = pid_maintain_angle();
	
	/*
	//To use Right thumbstick to control angle
	if(!(xbc_get_joy(XBC_JOY_RX) == 0 && xbc_get_joy(XBC_JOY_RY) == 0))
	{
		angle = int_arc_tan2(xbc_get_joy(XBC_JOY_RY), xbc_get_joy(XBC_JOY_RX));
		angle = angle - 90;
		if(angle < 0)
			angle = 360 + angle;
		angle = 360 - angle;
		if(angle == 360)
			angle = 0;
		
		wheel_base_set_target_pos((POSITION){get_pos()->x, get_pos()->y, 10*angle});
	
	}
	*/
	
	wheel_base_set_vel(dx, dy, dw);

	//Get acceleration modifiers for each wheel
	
	 //Set wheel accelerations
//		motor_set_acceleration(MOTOR_BOTTOM_RIGHT,(acc_mod * br)/100);
//		motor_set_acceleration(MOTOR_BOTTOM_LEFT,(acc_mod * bl)/100);
//		motor_set_acceleration(MOTOR_TOP_LEFT,(acc_mod * tl)/100);
//		motor_set_acceleration(MOTOR_TOP_RIGHT, (acc_mod * tr)/100);
	
	//y = r*cos(theta)
	//x = r*sin(theta)
	
		
	/*
	80 = up
	81= down
	4D = right
	51 = left
	*/
	//Function Keys 
	
	if (ROBOT=='C')
	 	robot_c_function_controls();
	if (ROBOT=='D')
		robot_d_function_controls();
	
	
	//RB and LB
	/*if(button_released(BUTTON_XBC_LB) > 15)
	{
		wheel_base_set_speed_mode(wheel_base_get_speed_mode() - 1);
	}
	else if(button_released(BUTTON_XBC_RB) >  15)
	{
		wheel_base_set_speed_mode(wheel_base_get_speed_mode() + 1);
	}	
	*/
	return true;
}

void robot_c_function_controls(void)
{
	// enable sensors
	if(button_pressed(BUTTON_XBC_RB))
		sensors_activated=1;
	else
		sensors_activated=0;
	
	//Racket Hit
	if(button_pressed(BUTTON_XBC_B))
		racket_hit();
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
  if (button_hold(BUTTON_XBC_XBOX, 10, 1)) {
    force_terminate = true;
    return;
  } else if (button_released(BUTTON_XBC_XBOX)) {
    force_terminate = false;
  }
  
	// enable sensors
	if(button_pressed(BUTTON_XBC_RB))
		sensors_activated=1;
	else
		sensors_activated=0;
	
	//Racket Hit
	if(button_pressed(BUTTON_XBC_A))
		racket_hit();
	//Calibrate
	else if (button_pressed(BUTTON_XBC_LB) && button_pressed(BUTTON_XBC_B))
		fake_serve_start();
	else if(button_pressed(BUTTON_XBC_B))
		serve_start();
	else if(button_pressed(BUTTON_XBC_X))//Not essential
		serve_calibrate();
	// change serve varibales
	
	if (button_pressed(BUTTON_XBC_RB)) {
		gyro_pos_set(get_pos()->x, get_pos()->y, 0);
		accumulated_omega = target_angle = 0;
	}
	
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
	
	
	if (button_pressed(BUTTON_XBC_START))
	{
		serve_free();
	}
	if (serve_pneu_button_enabled && gpio_read_input(&PE5))
	{
		serve_pneu_button_enabled=0;
		toggle_serve_pneu();
	}
	if (!gpio_read_input(&PE5))
	{
		serve_pneu_button_enabled=1;
	}
	
	/**
	* NEC controls
	
	*/
	if (nec_get_msg(0)->address==0x40 && nec_get_msg(0)->command==0x05)
	{
		racket_hit();
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
				racket_hit();
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
	motor_set_acceleration(RACKET, 3000);
  // Send the acceleration data
	wheel_base_tx_acc();
	serve_free();
	gpio_init(&PD9,  GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);		// Serve pneumatric GPIO Robot D, GEN2
	gpio_init(&PE15,  GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);		// Serve pneumatric GPIO Robot D, GEN2. MOSFET burnt
	gpio_init(&PD10, GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);		// pneu matic GPIO
	
	
	gpio_init(&PE11, GPIO_Speed_10MHz, GPIO_Mode_IPU, 1);	// Mechanical switch ROBOT D Gen2
	gpio_init(&PE5, GPIO_Speed_10MHz, GPIO_Mode_IPU, 1);	// Shuttlecock Holder button for ROBOT D Gen2

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
				tick_skip_count+=1;
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
			if (!force_terminate) {
				// wheel_base update
				wheel_base_update();
			} else {
				wheel_base_set_vel(0, 0, 0);
				motor_set_vel(MOTOR_BOTTOM_RIGHT, 0, OPEN_LOOP);
				motor_set_vel(MOTOR_BOTTOM_LEFT, 0, OPEN_LOOP);
				motor_set_vel(MOTOR_TOP_RIGHT, 0, OPEN_LOOP);
				motor_set_vel(MOTOR_TOP_LEFT, 0, OPEN_LOOP);
				serve_free();
			}
				
			if (ticks_img % 250 == 1) {
				// Every 250 ms (4 Hz)
				//battery_adc_update();
				//send_string_s("hello\n", 6);
				send_string("hi");
			}
			
			
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
				
				if(ROBOT=='C')
				{ 
						tft_prints(0,2,"Acc: %d", acc_mod);
						tft_prints(0,3,"Angle: %d", angle);
				}
				else
				{
					tft_prints(0,2,"Encoder: %d", get_encoder_value(RACKET));
					tft_prints(0,3,"Serve_delay: %d",serve_get_delay());
					tft_prints(0,4,"Racket: %d", serve_get_vel());
          tft_prints(0,5,"Target: %d", wheel_base_get_target_pos().angle);
					tft_prints(0,6,"Angle: %d", get_pos()->angle);
				}
					 
        
				tft_update();
			}
			
		}
		
	}	
}
