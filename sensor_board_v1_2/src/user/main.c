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
	
	//button_init();
	led_init();
	//tft_init(0, WHITE, BLACK, RED);
	//gyro_init();

  //battery_init(); 
  //adc_init();
	can_init();
	can_rx_init();
  //can_motor_init();
	//NEC_init();
	
	//bluetooth_init();
  //mb1240_init();
  //xbc_mb_init(XBC_BLUETOOTH_FIRST); 
	//wheel_base_init();
  us_init();
	us_proc_init();
  nec_init();
  /** For debugging **/
	uart_init(COM1, 115200);
	uart_printf_enable(COM1);
	//uart_tx(COM2, "Init...");
	led_control(LED_SIG1 | LED_SIG2 | LED_SIG3 | LED_SIG4, LED_ON);
	buzzer_play_song(START_UP, 120, 0);
	printf("COM1 init...");
	
	while (get_seconds() < 1) {}

	led_control(LED_SIG1 | LED_SIG2 | LED_SIG3 | LED_SIG4, LED_OFF);
	
	while(1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			if (ticks_img % 20 == 0) {
				nec_update();
			}
			
			
			
			if (ticks_img % 5 == 2) {
				us_proc_update();
			}
			
			led_control(LED_SIG1, (LED_STATE) !gpio_read_input(&PA0));

			if (ticks_img % 10 == 3) {
				for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
					if (us_in_range(i)) {
						buzzer_set_note_period(get_note_period(NOTE_C, 7) + us_get_distance(i));
						buzzer_control(2, 100);
					}
				}
			}
			
			if (ticks_img % 50 == 4) {
				for (u8 i = 0; i < NEC_DEVICE_COUNT; ++i) {
					nec_can_tx(i);
				}
			}
			
			if (ticks_img % 50 == 5) {
				nec_printf();
			}

		}
	}
	
}

