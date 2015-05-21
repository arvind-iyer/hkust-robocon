#include "main.h"
#include "ks103.h"

static u16 ticks_img = (u16) -1;

static u8 ks100_mode = 0;


void ks100_decode(void)
{
	static u8 y = 2;
	
	//tft_prints(0, y
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
	tft_init(0, WHITE, BLACK, RED);
	//gyro_init();

  battery_init(); 
  //adc_init();
  //can_init();
	//can_rx_init();
  //can_motor_init();
	ks103_init();
	
	//bluetooth_init();
  //mb1240_init();
  //xbc_mb_init(XBC_BLUETOOTH_FIRST); 
	//wheel_base_init();
  //us_mb_init();
  //nec_mb_init();
  /** For debugging **/
  //uart_init(COM1, 115200);
  //uart_printf_enable(COM1);
  
  
  system_start(1200);

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
  
	menu(0, true);
	*/
	

	//uart_rx_init(COM1, );
	
	while(1) {
	
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 20 == 0) {
				button_update();
				
				if (button_pressed(BUTTON_1) == 1) {
					bool flag = ks103_tx(0);
					if (flag) {
						CLICK_MUSIC;
					}
					
				}
			}
			
			if (ticks_img % 40 == 5) {
				ks103_tx_force(0);
				ks103_tx_force(1);
			}
			if (ticks_img % 20 == 10) {
				tft_clear();
				draw_top_bar();
				

				tft_prints(0,1,"KS100 TEST");
				for (u8 id = 0; id < KS103_COUNT; ++id) {
					tft_prints(0,2 + id,"%4d  (%d)  %d", ks103_get(id)->range, ks103_get(id)->state, ks103_get(id)->sample_count);
					if (ks103_get(id)->range >= 20 && ks103_get(id)->range <= 1500) {
						buzzer_set_note_period(get_note_period(NOTE_C, 6) + ks103_get(id)->range); 
						buzzer_control(3, 100);
					}
				}
				
				
				tft_update();
			}
		}
	}
}

