#include "interface.h"

void system_start(u16 duration)
{
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_ON);
	tft_clear();
	
	// Start-up battery check
	battery_adc_update();
	switch(battery_check()) {
		case BATTERY_USB:
		case BATTERY_OKAY:
			// DO NOTHING
			buzzer_play_song(START_UP, 120, 0);
			tft_prints(0, 0, "Battery okay!");
			tft_prints(0, 1, "Battery: %s", get_voltage_string());
		break;
		case BATTERY_LOW:
			buzzer_set_note_period(get_note_period(NOTE_E, 7));
			buzzer_control(5, 100);
			tft_prints(0, 0, "LOW BATTERY!");
			tft_prints(0, 1, "Battery: %s", get_voltage_string());
		break;
		case BATTERY_SUPER_LOW:
			buzzer_set_note_period(get_note_period(NOTE_C, 7));
			buzzer_control(10, 50);
			tft_prints(0, 0, "NO BATTERY!");
			tft_prints(0, 1, "Battery: %s", get_voltage_string());
			tft_update();
			while(1) {
				_delay_ms(200);
				tft_prints(0, 0, "            ");
				tft_update();
				_delay_ms(200);
				tft_prints(0, 0, "NO BATTERY!");
				tft_update();
			}
		
	}
	tft_update();
	
	_delay_ms(duration);
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_OFF);
	
}


void battery_regular_check(void)
{
	switch(battery_check()) {
		case BATTERY_USB:
		case BATTERY_OKAY:
			// DO NOTHING
		break;
		case BATTERY_LOW:
			buzzer_set_note_period(get_note_period(NOTE_E, 7));
			buzzer_control(2, 100);
		break;
		case BATTERY_SUPER_LOW:
			buzzer_set_note_period(get_note_period(NOTE_E, 7));
			buzzer_control(6, 200);
		break;
	}
		
}

