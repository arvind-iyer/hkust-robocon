#include "main.h"

static u16 ticks_img 	= (u16)-1;
static u16 seconds_img = (u16)-1;

int main(void)
{
	/* Initialization */
	/* Note: Init order is important! */
	ticks_init();
	buzzer_init();
	//button_init();
	led_init();
	tft_init(0, BLACK, WHITE, RED);
	gyro_init();
	battery_adc_init();
	can_init();
	can_rx_init();
	bluetooth_init();
	wheel_base_init();
	wheel_base_pid_init();
	
	//usart_init(COM1, 115200);


	system_start(1200);
	robocon_main();

		while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 7) {
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
				tft_prints(0, 6, "Pos: %-4d,%-4d", get_pos()->x, get_pos()->y);
				tft_prints(0, 7, "Ang: %-4d", get_pos()->angle);
				tft_prints(0, 8, "%d%d%d%d%d%d%d%d%d%d", 
				gpio_read_input(BUTTON_J1_UP_GPIO), gpio_read_input(BUTTON_J1_DOWN_GPIO), gpio_read_input(BUTTON_J1_LEFT_GPIO), gpio_read_input(BUTTON_J1_RIGHT_GPIO), gpio_read_input(BUTTON_J1_CENTER_GPIO),
				gpio_read_input(BUTTON_J2_UP_GPIO), gpio_read_input(BUTTON_J2_DOWN_GPIO), gpio_read_input(BUTTON_J2_LEFT_GPIO), gpio_read_input(BUTTON_J2_RIGHT_GPIO), gpio_read_input(BUTTON_J2_CENTER_GPIO));
				tft_prints(0, 9, "Time: %d'%03d\"", get_seconds(), get_ticks());
				tft_update();
			}
			
			
		}
	}	
		
}



