#include "main.h"

static u16 ticks_img 	= (u16)-1;
static u16 seconds_img = (u16)-1;

int main(void)
{
	/* Initialization */
	/* Note: Init order is important! */
	ticks_init();
	buzzer_init();
	button_init();
	led_init();
	tft_init(0, WHITE, BLACK, RED);
	gyro_init();
	battery_adc_init();
	can_init();
	can_rx_init();
	bluetooth_init();
	wheel_base_init();
	wheel_base_pid_init();
	
	//usart_init(COM1, 115200);


	system_start("Robocon 2015  Min System 1.0.0", 1200);
	
	menu_add("Your program", robocon_main);
	menu_add("Position test", position_test);
	menu_add("Motor test", 0);
	menu_add("Battery test", battery_test);
	menu_add("Bluetooth test", bluetooth_test);
	menu_add("Buzzer test", 0);
	menu_add("Button test", 0);
	menu_add("ASCII test", ascii_test);
	menu(0);
	robocon_main();

		while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 7) {
				button_update();
				// Every 50 ms (20 Hz)
				/** Warning: try not to do many things after tft_update(), as it takes time **/
				WHEEL_BASE_VEL vel = wheel_base_get_vel();
				tft_clear();
				tft_prints(0, 0, "Battery: %s", get_voltage_string());
				tft_prints(0, 1, "ADC: %d", get_battery_adc());
				tft_prints(0, 2, "VX: %d", vel.x);
				tft_prints(0, 3, "VY: %d", vel.y);
				tft_prints(0, 4, "VW: %d", vel.w);
				tft_prints(0, 5, "Speed: %d", wheel_base_get_speed_mode());
				tft_prints(0, 6, "(%-4d,%-4d,%-4d)", get_pos()->x, get_pos()->y, get_pos()->angle);

				//tft_prints(0, 8, "%lld", hello);
				tft_prints(0, 7, "%d%d%d%d%d%d%d%d%d%d", \
				button_pressed(BUTTON_JS1_UP) % 10, button_pressed(BUTTON_JS1_DOWN) % 10, button_pressed(BUTTON_JS1_LEFT) % 10, button_pressed(BUTTON_JS1_RIGHT) % 10, button_pressed(BUTTON_JS1_CENTER) % 10, \
				button_pressed(BUTTON_JS2_UP) % 10, button_pressed(BUTTON_JS2_DOWN) % 10, button_pressed(BUTTON_JS2_LEFT) % 10, button_pressed(BUTTON_JS2_RIGHT) % 10, button_pressed(BUTTON_JS2_CENTER) % 10);
				
				tft_prints(0, 7, "%d%d%d%d%d%d%d%d%d%d", \
				button_released(BUTTON_JS1_UP) % 10, button_released(BUTTON_JS1_DOWN) % 10, button_released(BUTTON_JS1_LEFT) % 10, button_released(BUTTON_JS1_RIGHT) % 10, button_released(BUTTON_JS1_CENTER) % 10, \
				button_released(BUTTON_JS2_UP) % 10, button_released(BUTTON_JS2_DOWN) % 10, button_released(BUTTON_JS2_LEFT) % 10, button_released(BUTTON_JS2_RIGHT) % 10, button_released(BUTTON_JS2_CENTER) % 10);
	

				tft_prints(0, 9, "Time: %d'%03d\"", get_seconds(), get_ticks());
				tft_update();
			}
			
			
		}
	}	
		
}



