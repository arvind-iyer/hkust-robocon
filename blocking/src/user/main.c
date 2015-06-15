#include "main.h"
// PA5, PA6, PA7

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
	//buzzer_init();
	gpio_init(BUZZER_GPIO, GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);
	gpio_init(LED_D1_GPIO, GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);
	button_init();
	//led_init();
	//tft_init(2, WHITE, BLACK, RED);
	//gyro_init();

  //battery_init(); 
  //adc_init();
  //can_init();
	//can_rx_init();
  //can_motor_init();
	
	//bluetooth_init();
  //mb1240_init();
  //xbc_mb_init(XBC_BLUETOOTH_FIRST); 
	//wheel_base_init();
  //us_mb_init();
  //nec_mb_init();
  /** For debugging **/
  //uart_init(COM1, 115200);
  //uart_printf_enable(COM1);
  //robocon_init();
  
	gpio_init(&PA5, GPIO_Speed_10MHz, GPIO_Mode_IPD, 1);
	gpio_init(&PA6, GPIO_Speed_10MHz, GPIO_Mode_IPD, 1);
	gpio_init(&PA7, GPIO_Speed_10MHz, GPIO_Mode_IPD, 1);
	
	
	//buzzer_control(1, 1000);
	
  //system_start(1200);
	
	u16 ticks_img = (u16) -1;
	
	u8 flag = 0;
	while(1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (get_seconds() == 0 && ticks_img < 500) {
				gpio_write(BUZZER_GPIO, 1);
				gpio_write(LED_D1_GPIO, 1);
				//led_control(LED_D1 | LED_D2 | LED_D3, LED_ON);
				continue;
			}
			
			if (gpio_read_input(&PA5) == 1 || gpio_read_input(&PA6) == 1 || gpio_read_input(&PA7) == 1) {
				flag = 1;
			}
			
			if (flag) {
				gpio_write(BUZZER_GPIO,1);
				//led_control(LED_D1 | LED_D2 | LED_D3, LED_ON);
			} else {
				gpio_write(BUZZER_GPIO,0);
				led_control(LED_D1 | LED_D2 | LED_D3, LED_OFF);
			}

			
			if (button_pressed(BUTTON_1) == 1 || button_pressed(BUTTON_2) == 1) {
				flag = 0;
			}
		}
	}
	
	/*
	menu_add("Your program", robocon_main);
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
  //menu_add("MB1240 test", mb1240_test);
  menu_add("Ultra. test", us_mb_test);
  
	menu(0, false);
	*/
}

