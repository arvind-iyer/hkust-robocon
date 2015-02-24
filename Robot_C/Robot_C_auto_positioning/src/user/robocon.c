#include "robocon.h"

//system values
static u16 ticks_img 	= (u16)-1;
static u32 last_sent_OS_time = 0;
static bool key_trigger_enable = true;

//pneumatic racket value
static bool current_pneumatic=0;


void wheel_base_joystick_control(void)
{
	//wheel_base_set_vel(0,0,0);
	if (button_pressed(BUTTON_JS2_LEFT))
	{
		wheel_base_joyStickCommandFlag_on();
		wheel_base_set_vel(-wheel_base_get_joystick_speed(),0,0);
		//commandReceivedFlag=1;
	}
	if (button_pressed(BUTTON_JS2_RIGHT))
	{
		wheel_base_joyStickCommandFlag_on();
		wheel_base_set_vel(wheel_base_get_joystick_speed(),0,0);
	}
	if (button_pressed(BUTTON_JS2_UP))
	{
		wheel_base_joyStickCommandFlag_on();
		wheel_base_set_vel(0,wheel_base_get_joystick_speed(),0);
	}
	if (button_pressed(BUTTON_JS2_DOWN))
	{
		wheel_base_joyStickCommandFlag_on();
		wheel_base_set_vel(0,-wheel_base_get_joystick_speed(),0);
	}
	if (button_pressed(BUTTON_JS1_RIGHT))
	{
		wheel_base_joyStickCommandFlag_on();
		wheel_base_set_vel(0,0,wheel_base_get_joystick_speed());
	}
	if (button_pressed(BUTTON_JS1_LEFT))
	{
		wheel_base_joyStickCommandFlag_on();
		wheel_base_set_vel(0,0,-wheel_base_get_joystick_speed());
	}
	if (button_pressed(BUTTON_JS1_CENTER)==1)
	{
		wheel_base_decrease_joystick_speed();
	}
	if (button_pressed(BUTTON_JS2_CENTER)==1)
	{
		wheel_base_increase_joystick_speed();
	}
}

static void pneumatic_control(bool data)		// 
{
	gpio_write(&PE9, !data);
}




static void handle_bluetooth_input(void)
{
	if (key_trigger_enable && !bluetooth_is_key_release())
	{
		last_sent_OS_time = get_full_ticks();
		key_trigger_enable = false;
		wheel_base_pid_off();
		switch (wheel_base_bluetooth_get_last_char())
		{
			case 'k':
				racket_hit();
			break;
			case 'l':
				racket_calibrate();
			break;
			case 'o':
				racket_lock();
			break;
			case 'p':
				racket_stop();
			break;
	 }
	}
	else if (bluetooth_is_key_release())
	{
		key_trigger_enable = true;
	}
	
}

void robocon_main(void)
{

//  static char last_key;
  // Send the acceleration data
	wheel_base_tx_acc();
	//racket_init();
	//racket_stop();
	gpio_init(&PE9, GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);		// pneumatic GPIO
	//register_special_char_function('m',print);
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 10 == 0) {
        wheel_base_update();	//wheel_base_update now also handles auto positioning system
				bluetooth_update();
        handle_bluetooth_input();
 
        button_update();
				//racket_update();
				// Every 10 ms (100 Hz)
        
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
				if(special_char_handler_bt_get_last_char() == 'k')
				{
					tft_prints(0,7, "HIT");
					tft_update();
				}
      }
			
			if (ticks_img % 100 == 3) {
				// Every 100 ms (10 Hz)
				wheel_base_tx_position();
			}
			
			if (ticks_img % 500 == 4) {
				led_control(LED_D3, (LED_STATE) (ticks_img == 0));
			}
			
			if (ticks_img % 50 == 5) {
				wheel_base_joystick_control();
				if (button_pressed(BUTTON_1) > 10 || button_pressed(BUTTON_2) > 10) {
					/** Stop the wheel base before return **/
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
				tft_prints(0, 2, "Speed: %d", wheel_base_get_speed_mode());				char s[3] = {wheel_base_bluetooth_get_last_char(), '\0'};
        if (s[0] == '[' || s[0] == ']') {
          // Replace "[" and "]" as "\[" and "\]"
          s[1] = s[0];
          s[0] = '\\';
        }
				
        tft_prints(0, 6, "Char: %s (%d) %c", s, wheel_base_bluetooth_get_last_char(), special_char_handler_bt_get_last_char());
				tft_prints(0,5, "Switch = %d", gpio_read_input(&PE3));
				tft_prints(0,8,"Encoder: %d", get_encoder_value(RACKET));
				tft_prints(0,7,"init: %d", get_init_enc());
				
				tft_update();
			}
			
		}
		
	}	
}
