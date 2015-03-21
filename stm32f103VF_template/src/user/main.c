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
  
  xbc_mb_init(XBC_BLUETOOTH_FIRST); 
	wheel_base_init();
  //ultrasonic_init();
  nec_init();

  
  
	menu_add("Your program", robocon_main);
	menu_add("Position test", position_test);
	menu_add("Motor test", motor_test);
	menu_add("Battery test", battery_test);
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
  menu_add("Ultra. test", ultra_test);
  
	menu(6, false);
	
}

