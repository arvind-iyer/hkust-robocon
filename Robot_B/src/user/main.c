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

  battery_init(); 
  adc_init();
  can_init();
	can_rx_init();
  can_motor_init();
	
	bluetooth_init();
  //mb1240_init();
  xbc_mb_init(XBC_BLUETOOTH_FIRST); 
	wheel_base_init();
	us_mb_init();
	gamepad_led_init();
	
  //nec_init();
  /** For debugging **/
  // uart_init(COM1, 115200);
  // uart_printf_enable(COM1);
	
	ultrasonic_init();
	
	
	racket_init();
  
  system_start(1200);
  
	menu_add("Robot B Main", robocon_main);
	menu_add("Position test", position_test);
	menu_add("Motor test", motor_test);
	menu_add("ADC test", adc_test);
  menu_add("ADC app test", adc_app_test);
	menu_add("Bluetooth test", bluetooth_test);
	menu_add("CAN test", can_test);
  menu_add("BT XBox test", bluetooth_xbc_test);
	menu_add("CAN XBox test", can_xbc_test);
  menu_add("XBox test", xbc_test);
	menu_add("Buzzer test", buzzer_test);
	menu_add("Button test", button_test);
	menu_add("ASCII test", ascii_test);
	menu_add("GPIO Pin test", gpio_pin_test);
	menu_add("UART test", uart_test);
	menu_add("NEC test", nec_mb_test);
	menu_add("Ultra. test", us_mb_test);
  
	menu(0, false);
	
}

