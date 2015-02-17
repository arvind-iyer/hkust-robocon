#include "main.h"

u16 ticks_img = (u16) -1;


void COM2_RX(u8 data)
{
  uart_tx_byte(UART_COM, data);
}

void rtc_interrupt(void)
{
  uart_tx(UART_COM, "%d\r\n", rtc_get_time());
}


/**
  * @brief Main function
  * @param None.
  * @retval None.
   */
int main(void)
{
	/* Initialization */
	/* Note: Init order is important! */
  //rtc_init();
  //rtc_set_time(0);
  //rtc_interrupt_init(rtc_interrupt);
	ticks_init();
	buzzer_init();
	button_init();
	led_init();
	tft_init(2, WHITE, BLACK, RED);
	gyro_init();
	battery_adc_init();
	//can_init();
	//can_rx_init();
  //can_motor_init();
	//bluetooth_init();
	wheel_base_init();
  //ultrasonic_init();
  uart_init(UART_COM, UART_COM_BR); 
	system_start(1200);
	uart_tx_byte(COM1, 0);
  uart_init(COM2, UART_COM_BR);
  uart_rx_init(COM2, COM2_RX);

  
  while (1) {
    if (ticks_img != get_ticks()) {
      ticks_img = get_ticks(); 
      if (ticks_img % 20 == 0) {
        button_update();
        
        if (button_pressed(BUTTON_E0) == 1) {
          uart_tx_byte(UART_COM, '0');
        }
        
        if (button_pressed(BUTTON_E1) == 1) {
          uart_tx_byte(UART_COM, '1');
        }
        
        if (button_pressed(BUTTON_E2) == 1) {
          uart_tx_byte(UART_COM, '2');
        }
        
        if (button_pressed(BUTTON_E3) == 1) {
          uart_tx_byte(UART_COM, '3');
        }
        
        if (button_pressed(BUTTON_E4) == 1) {
          uart_tx_byte(UART_COM, '4');
        }
        
        if (button_pressed(BUTTON_E5) == 1) {
          uart_tx_byte(UART_COM, '5');
        }
        
        if (button_pressed(BUTTON_E6) == 1) {
          uart_tx_byte(UART_COM, '6');
        }
        
        if (button_pressed(BUTTON_E7) == 1) {
          uart_tx_byte(UART_COM, '7');
        }
        
        if (button_pressed(BUTTON_E8) == 1) {
          uart_tx_byte(UART_COM, '8');
        }
        
        if (button_pressed(BUTTON_E9) == 1) {
          uart_tx_byte(UART_COM, '9');
        }
        
        if (button_pressed(BUTTON_E10) == 1) {
          uart_tx_byte(UART_COM, ' ');
        }
        
        if (button_pressed(BUTTON_E11) == 1) {
          uart_tx_byte(UART_COM, '\n');
        }


        if (button_pressed(BUTTON_E12) == 1) {
          uart_tx_byte(UART_COM, 'q');
        }
        
        if (button_pressed(BUTTON_E13) == 1) {
          uart_tx_byte(UART_COM, 'p');
        }
        
        if (button_pressed(BUTTON_E14) == 1) {
          uart_tx_byte(UART_COM, 's');
        }
        
        if (button_pressed(BUTTON_E15) == 1) {
          uart_tx_byte(UART_COM, 'r');
        }
        
        if (button_pressed(BUTTON_D9) == 1) {
          uart_tx_byte(UART_COM, 't'); 
        }
        
        if (button_pressed(BUTTON_D10) == 1) {
          uart_tx_byte(UART_COM, 't');
        }
        
      }
    }
  }
  
  /*
	menu_add("Your program", robocon_main);
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
  menu_add("Ultra. test", ultra_test);
  
	menu(0, false);
	*/
  
}

