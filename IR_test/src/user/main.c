#include "main.h"

#define	NEC_TX_ADDRESS	0xAB
#define	NEC_TX_COMMAND	0x30
#define	NEC_GPIO				((GPIO*) &PE5)

void nec_tx_init(void)
{
	gpio_init(NEC_GPIO, GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);
}


void nec_set(u8 i)
{
	gpio_write(NEC_GPIO, (BitAction) i);
}

void nec_set_delay(u8 i, u32 delay_us)
{
	nec_set(i);
	_delay_us(delay_us);
}

void nec_tx_byte(u8 data)
{
	for (u8 i = 0; i < 8; ++i) {
		u8 bit = data & 0x01;
		if (bit) {
			nec_set_delay(1, 560);
			nec_set_delay(0, 1690);
		} else {
			nec_set_delay(1, 560);
			nec_set_delay(0, 560);
		}
		data >>= 1;
	}
}

void nec_tx(u8 address, u8 command)
{
	// Start-up burst
	nec_set_delay(1, 9000);
	nec_set_delay(0, 4500);
	
	u8 data = 0;
	// Address
	nec_tx_byte(address);
	nec_tx_byte(~address);
	
	nec_tx_byte(command);
	nec_tx_byte(~command);
	
	nec_set_delay(1, 560);
	nec_set_delay(0, 20000);
	
	//nec_set_delay(1, 8500);
	//nec_set_delay(0, 2000);
	
	//nec_set_delay(1, 560);
	//nec_set_delay(0, 90000);
	//nec_set_delay(0, 3000);
	//nec_set_delay(1, 3000);
}

void nec_rx(u8 data)
{
	static u8 last_data = 0;
	if (data == 0x31 && last_data == 0x42) {
		buzzer_control_note(3, 100, NOTE_C, 7);
	}
	
	last_data = data;
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
	ticks_init();
	buzzer_init();
	
	button_init();
	led_init();
	tft_init(2, WHITE, BLACK, RED);
	//gyro_init();

  battery_init(); 
  adc_init();
  //can_init();
	//can_rx_init();
  //can_motor_init();
	
	bluetooth_init();
  //mb1240_init();
  //xbc_mb_init(XBC_BLUETOOTH_FIRST); 
	//wheel_base_init();
  //us_mb_init();
  //nec_mb_init();
  /** For debugging **/
  //uart_init(COM1, 115200);
  //uart_printf_enable(COM1);
  //robocon_init();
	
	nec_tx_init();
	
  nec_set(0);
	
	uart_init(COM1, 100);
	uart_init(COM3, 100);
	uart_rx_init(COM3, nec_rx);
  //dsystem_start(1200);
	
	//gpio_init(&PE4, GPIO_Speed_2MHz, GPIO_Mode_IPD, 1);
	
	
	
	
	u16 ticks_img = (u16) -1;
	u32 count = 0;
	
	while (1) {
		nec_set(!gpio_read_input(&PA9));
		

		
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 500 == 0) {
				//nec_set(ticks_img == 0); 
			}
			
			if (ticks_img % 20 == 0) {
				tft_clear();
				tft_prints(0, 0, "%d", gpio_read_output(&PA9));
				tft_prints(0, 1, "%x", USART_ReceiveData(USART3));
				tft_update();
			}
			
			if (ticks_img % 100 == 5) {
				uart_tx_byte(COM1, 0x42);
			}
			
			if (ticks_img % 100 == 55) {
			
				uart_tx_byte(COM1, 0x31);
			}
		}
	
	
		/*
		nec_tx(NEC_TX_ADDRESS, NEC_TX_COMMAND);
		buzzer_control_note(1, 10, NOTE_Ab, 5);
		++count; 
		tft_clear();
		tft_prints(0, 0, "%d", count);
		tft_update();
		*/
	}
	
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
	
}

