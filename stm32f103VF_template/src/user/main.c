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
	menu_add("Motor test", motor_test);
	menu_add("Battery test", battery_test);
	menu_add("Bluetooth test", bluetooth_test);
	//menu_add("CAN test", can_test);
	//menu_add("XBox test", xbox_test);
	//menu_add("Buzzer test", buzzer_test);
	menu_add("Button test", button_test);
	menu_add("ASCII test", ascii_test);
	
	menu(0);
	robocon_main();
	
		
}



