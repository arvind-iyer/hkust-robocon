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
	can_rx_init();
	bluetooth_init();
	wheel_base_init();
	wheel_base_pid_init();
	
	//usart_init(COM1, 115200);

	system_start(1200);
	robocon_main();
		
}



