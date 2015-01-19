#include "main.h"

static u16 ticks_img 	= (u16)-1;
static u16 seconds_img = (u16)-1;

int main(void)
{
	/* Initialization */
	/* Note: Init order is important! */
	ticks_init();
	buzzer_init();
	led_init();
	tft_init(0, BLACK, WHITE, RED);
	gyro_init();
	battery_adc_init();
	can_init();
	bluetooth_init();
	wheel_base_init();
	//usart_init(COM1, 115200);

	system_start(1200);

	robocon_main();
	
	// USART Debugging
	//	usart_tx_byte(COM1, 'A');
	//	usart_tx_byte(COM1, 'B');
	//	usart_tx_byte(COM1, 'C');
	//	usart_tx_byte(COM1, 'D');
	
	
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			if (ticks_img % 50 == 0) {
				tft_clear();
				tft_prints(0,0,"USART_TEST: %d", get_seconds());
				tft_update();
			}
		}
	}
		
}



