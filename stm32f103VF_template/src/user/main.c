#include "main.h"


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
	tft_init(2, WHITE, BLACK, RED);
	gyro_init();
	battery_adc_init();
	can_init();
	can_rx_init();
  can_motor_init();
	bluetooth_init();
	wheel_base_init();
	wheel_base_pid_init();
  us_mb_init();
	special_char_handler_init();
	racket_init();
  xbc_mb_init(XBC_BLUETOOTH_FIRST);
	system_start(1200);
	
	menu_add("Robot A Prog", robocon_main);
	menu_add("Position test", position_test);
	menu_add("Motor test", motor_test);
	menu_add("Battery test", battery_test);
	menu_add("Bluetooth test", bluetooth_test);
	menu_add("CAN test", can_test);
	menu_add("XBox test", xbc_test);
	menu_add("Buzzer test", buzzer_test);
	menu_add("Button test", button_test);
	menu_add("ASCII test", ascii_test);
	menu_add("GPIO Pin test", gpio_pin_test);
	menu_add("UART test", uart_test);
  menu_add("Ultra. test", us_mb_test);
  
	menu(0, false);
	
}

