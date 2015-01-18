#include "main.h"

u16 ticks_img 	= (u16)-1;
u16 seconds_img = (u16)-1;



int main(void)
{
	ticks_init();
	buzzer_init();
	led_init();
	tft_init(0, BLACK, WHITE, RED);
	gyro_init();
	battery_adc_init();
	can_init();
	
	bluetooth_init();
	wheel_base_init();
	
	buzzer_play_song(START_UP, 120, 0);
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_ON);
	_delay_ms(1000);
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_OFF);
	
	//bluetooth_tx("Hello world\r\n");

	// ADC Battery Check
	/* TODO: Place this in "system.c" */
	{
		switch(battery_check()) {
			case BATTERY_USB:
			case BATTERY_OKAY:
				// DO NOTHING
			break;
			case BATTERY_LOW:
				buzzer_set_note_period(get_note_period(NOTE_E, 7));
				buzzer_control(5, 100);
			break;
			case BATTERY_SUPER_LOW:
				buzzer_set_note_period(get_note_period(NOTE_E, 7));
				buzzer_control(10, 50);
			break;
		}
	}
	
	_delay_ms(100);
		
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			

						
			if (ticks_img % 10 == 0) {
				bluetooth_update();
				wheel_base_update();
			}
			
			if (ticks_img % 250 == 1) {
				battery_adc_update();
			}

			
			if (ticks_img % 10 == 3) {
				WHEEL_BASE_VEL vel = wheel_base_get_vel();
				tft_clear();
				tft_prints(0, 0, "Battery: %02d.%02dV", get_voltage() / 100, get_voltage() % 100);
				tft_prints(0, 1, "ADC: %d", get_battery_adc());
				tft_prints(0, 2, "VX: %d", vel.x);
				tft_prints(0, 3, "VY: %d", vel.y);
				tft_prints(0, 4, "VW: %d", vel.w);
				tft_prints(0, 5, "Speed: %d", wheel_base_get_speed_mode());
				tft_prints(0, 6, "Pos: %-4d,%-4d", get_X(), get_Y());
				tft_prints(0, 7, "Ang: %-4d", get_angle());
				tft_prints(0, 9, "Time: %d'%03d\"", get_seconds(), get_ticks());
				tft_update();
			}
			
			if (ticks_img % 100 == 6) {
				//wheel_base_send_position();
			}
		}
	}
}


