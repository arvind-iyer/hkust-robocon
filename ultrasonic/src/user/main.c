#include "main.h"
#include "us_proc.h"


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
	tft_init(0, WHITE, BLACK, RED);
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
  us_init();
	us_proc_init();
  //nec_init();
  /** For debugging **/
	uart_init(COM2, 115200);
	uart_tx(COM2, "Init...");
	buzzer_play_song(START_UP, 120, 0);
	

	while(1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			if (ticks_img % 5 == 3) {
				us_proc_update();
			}
			
			if (ticks_img % 10 == 0) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "ULTRASONIC(%d)", us_get_speed());
				for (u8 i = 0, x = 0, y = 2; i < US_DEVICE_COUNT; ++i, ++y) {
					if (y >= 10) {x += 7; y = 2;}
					tft_prints(x, y, "%4d[%d]", us_get_distance(i), us_proc_in_range_time(i));
					
					if (us_in_range(i)) {
						buzzer_set_note_period(get_note_period(NOTE_C, 7) + us_get_distance(i));
						buzzer_control(2, 100);
					}
				}
				tft_update();
				
				
			}

		}
	}
	
}

