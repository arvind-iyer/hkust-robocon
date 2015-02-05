
#include "robocon.h"

static u16 ticks_img 	= (u16)-1;
static u16 current_servo_PWM = 1400; // from 0 to 1000

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
	
	/***CONTROL SERVO, IT WORKS. Range: from 900 to 2000 (0.9ms to 2ms). Range is set to 1000 to 1600 for this robot****/
	/****value 1500 is roughly middle-right on robot C*****/
	if (button_pressed(BUTTON_JS1_UP)==1)
	{                                         
		if (current_servo_PWM<=1500)
			current_servo_PWM+=150;
		servo_control(SERVO4,current_servo_PWM);
		servo_control(SERVO3,current_servo_PWM);
		servo_control(SERVO2,current_servo_PWM);
		servo_control(SERVO1,current_servo_PWM);
	}
	if (button_pressed(BUTTON_JS1_DOWN)==1)	
	{
		if (current_servo_PWM>=1300)
			current_servo_PWM-=150;
		servo_control(SERVO4,current_servo_PWM);
		servo_control(SERVO3,current_servo_PWM);
		servo_control(SERVO2,current_servo_PWM);
		servo_control(SERVO1,current_servo_PWM);
		
	}
	
	
	
	
}


void robocon_main(void)
{
  // Send the acceleration data
	wheel_base_tx_acc();
	gpio_init(&PE0, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, RCC_APB1Periph_TIM4);
	//use gpio_write(&PE0, 0 or 1); for pneumatic (piston)
	//use servo_control(SERVO4, 0 to 1000); for servo control 
	
	u8 pneu = 0;
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 10 == 0) {
				// Every 10 ms (100 Hz)
				bluetooth_update();
				wheel_base_update();
			}
			
			if (ticks_img % 250 == 1) {
				// Every 250 ms (4 Hz)
				battery_adc_update();
			}
			//if(get_seconds() % 4 == 0)
			//	servo_control(SERVO4,1000);
			//else if(get_seconds() % 2 == 0)
			//	servo_control(SERVO4,0);
			
			if (get_seconds() % 10 == 2 && ticks_img == 2) {
				// Every 10 seconds (0.1 Hz)
				battery_regular_check();
			}
			/*TO TEST PISTON: Works!*/
			if(get_seconds() % 2 == 0)
			{
				gpio_write(&PE0, pneu++%2);
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
				tft_prints(0, 2, "Speed: %d", wheel_base_get_speed_mode());
				tft_prints(0, 3, "(%-4d,%-4d,%-4d)", get_pos()->x, get_pos()->y,get_pos()->angle);
				tft_prints(0, 4, "%s", wheel_base_get_pid_flag() ? "[AUTO]" : "MANUAL");
				tft_prints(0, 5, "(%-4d,%-4d,%-4d)", wheel_base_get_target_pos().x, wheel_base_get_target_pos().y, wheel_base_get_target_pos().angle);
				tft_prints(0, 7, "Servo PWM = %-4d", current_servo_PWM);
				char s[3] = {wheel_base_bluetooth_get_last_char(), '\0'};
        if (s[0] == '[' || s[0] == ']') {
          // Replace "[" and "]" as "\[" and "\]"
          s[1] = s[0];
          s[0] = '\\';
        }
        tft_prints(0, 6, "Char: %s (%d)", s, wheel_base_bluetooth_get_last_char());
        
				tft_update();
			}
			
		}
		
	}	
}
