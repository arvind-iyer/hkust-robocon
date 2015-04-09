#include "main.h"

u16 ticks_img = (u16)-1;

/**
  * @brief Main function
  * @param None.
  * @retval None.
   */
int main(void)
{
	/* Initialization */
	/* Note: Init order is important! */
	ticks_init();
	buzzer_init();
	
	button_init();
	led_init();
	tft_init(1, WHITE, BLACK, RED);
	//gyro_init();

  battery_init(); 
  adc_init();
  can_init();
	can_rx_init();
  //can_motor_init();
	
	//bluetooth_init();
  //mb1240_init();
  //xbc_mb_init(XBC_BLUETOOTH_FIRST); 
	//wheel_base_init();
  us_init(US_SYNC);
  //nec_init();
  /** For debugging **/

	buzzer_play_song(START_UP, 120, 0);
	
  
	while(1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			if (ticks_img % 10 == 0) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "ULTRASONIC (%s)", us_get_mode() == US_TAKE_TURN ? "TAKE TURN" : us_get_mode() == US_SYNC ? "SYNC" : "Normal");
				for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
					tft_prints(0, 2 + i, "[%d] %d %4d %4d %2d", i, us_get_state(i), us_get_pulse(i), us_get_distance(i), us_get_speed(i));
					
					if (us_get_distance(i) > 10 && us_get_distance(i) <= 1000) {
						buzzer_set_note_period(get_note_period(NOTE_C, 7) + us_get_distance(i));
						buzzer_control(3, 100);
					}
				}
				tft_update();
			}
		}
	}
	
}

