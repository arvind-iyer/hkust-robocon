#include "main.h"

u16 ticks_img = (u16) -1;

void NEC_RX(NEC_Msg msg)
{
	buzzer_control_note(3, 100, NOTE_C, 7);
	if (msg.address == NEC_REMOTE_LG.address) {
		if (msg.command >= NEC_REMOTE_LG.commands.digit0 && msg.command <= NEC_REMOTE_LG.commands.digit9) {
			u8 digit = msg.command - NEC_REMOTE_LG.commands.digit0;
			buzzer_control_note(1, 200, NOTE_D, 7); 
			uart_tx(UART_COM, "%d", digit);
		} else if (msg.command == NEC_REMOTE_LG.commands.ok) {
			buzzer_control_note(1, 200, NOTE_G, 7);
			uart_tx(UART_COM, "\n");	// 
		} else if (msg.command == NEC_REMOTE_LG.commands.power) {
			buzzer_control_note(1, 300, NOTE_G, 7);
			uart_tx(UART_COM, " ");	// Start with pre-count
		} else if (msg.command == NEC_REMOTE_LG.commands.back || msg.command == NEC_REMOTE_LG.commands.exit) {
			buzzer_control_note(1, 200, NOTE_B, 7);
			uart_tx(UART_COM, "q");	// Stop timer
		} else if (msg.command == NEC_REMOTE_LG.commands.red) {
			buzzer_control_note(1, 200, NOTE_B, 7);
			uart_tx(UART_COM, "t");	// Timeout
		} else if (msg.command == NEC_REMOTE_LG.commands.green) {
			buzzer_control_note(1, 200, NOTE_B, 7);
			uart_tx(UART_COM, "p");	// Preparation
		} else if (msg.command == NEC_REMOTE_LG.commands.yellow) {
			buzzer_control_note(1, 200, NOTE_B, 7);
			uart_tx(UART_COM, "s");	// serve
		} else if (msg.command == NEC_REMOTE_LG.commands.blue) {
			buzzer_control_note(1, 200, NOTE_B, 7);
			uart_tx(UART_COM, "r");	// setting
		} else if (msg.command == NEC_REMOTE_LG.commands.menu) {
			buzzer_control_note(1, 200, NOTE_G, 6);
			uart_tx(UART_COM, "timer");
		}	else if (msg.command == NEC_REMOTE_LG.commands.home) {
			buzzer_control_note(1, 200, NOTE_G, 6);
			uart_tx(UART_COM, "c");
		}	else if (msg.command == NEC_REMOTE_LG.commands.mute) {
			buzzer_control_note(1, 200, NOTE_G, 6);
			uart_tx(UART_COM, "volume0");
		} else if (msg.command == NEC_REMOTE_LG.commands.volume_down) {
			buzzer_control_note(1, 200, NOTE_G, 6);
			uart_tx(UART_COM, "volume2");
		} else if (msg.command == NEC_REMOTE_LG.commands.volume_up) {
			buzzer_control_note(1, 200, NOTE_G, 6);
			uart_tx(UART_COM, "volume3");
		} else if (msg.command == NEC_REMOTE_LG.commands.channel_up) {
			buzzer_control_note(1, 200, NOTE_G, 6);
			uart_tx(UART_COM, "set_hour");
		} else if (msg.command == NEC_REMOTE_LG.commands.channel_down) {
			buzzer_control_note(1, 200, NOTE_G, 6);
			uart_tx(UART_COM, "set_minute");
		} else if (msg.command == NEC_REMOTE_LG.commands.input) {
			buzzer_control_note(1, 200, NOTE_F, 6);
			uart_tx(UART_COM, "~");		// Random
		} else if (msg.command == NEC_REMOTE_LG.commands.up) {
			buzzer_control_note(1, 200, NOTE_F, 6);
			uart_tx(UART_COM, "mario_start");		// Random
		} else if (msg.command == NEC_REMOTE_LG.commands.down) {
			buzzer_control_note(1, 200, NOTE_F, 6);
			uart_tx(UART_COM, "mario_end");		// Random
		} else if (msg.command == NEC_REMOTE_LG.commands.left) {
			buzzer_control_note(1, 200, NOTE_F, 6);
			uart_tx(UART_COM, "birthday");		// Random
		} else if (msg.command == NEC_REMOTE_LG.commands.right) {
			buzzer_control_note(1, 200, NOTE_F, 6);
			uart_tx(UART_COM, "mario_song");		// Random
		} else {
			//#warning debug
			//uart_tx(UART_COM, "%X", msg.command); 
		}
	}
}

void COM2_RX(u8 data)
{
  uart_tx_byte(UART_COM, data);
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
	//gyro_init();
	//battery_adc_init();
	nec_init();
	//can_init();
	//can_rx_init();
  //can_motor_init();
	//bluetooth_init();
	//wheel_base_init();
  //ultrasonic_init();
	
  uart_init(UART_COM, UART_COM_BR); 
	
	uart_tx(COM1, 0);
  uart_init(COM2, UART_COM_BR);
  uart_rx_init(COM2, COM2_RX);
	
	system_start(1200);
	//uart_printf_enable(COM1);
  u8 mode = gpio_read_input(&PB9); // True for clock, false for timer
	
	nec_set_handler(NEC_RX);
	
  while (1) {
    if (ticks_img != get_ticks()) {
      ticks_img = get_ticks(); 
			led_control(LED_D1, (LED_STATE) !gpio_read_input(NEC_GPIO));
			
      if (ticks_img % 20 == 0) {
        button_update();
				
				if (mode ^ gpio_read_input(&PB9)) {	// Change
					mode = !mode;
					if (mode) {
						uart_tx(UART_COM, "c");
					} else {
						uart_tx(UART_COM, "timer");
					}
				}
        
        if (button_pressed(BUTTON_E0) == 1) {
          uart_tx(UART_COM, "0");
        }
        
        if (button_pressed(BUTTON_E1) == 1) {
          uart_tx(UART_COM, "1");
        }
        
        if (button_pressed(BUTTON_E2) == 1) {
          uart_tx(UART_COM, "2");
        }
        
        if (button_pressed(BUTTON_E3) == 1) {
          uart_tx(UART_COM, "3");
        }
        
        if (button_pressed(BUTTON_E4) == 1) {
          uart_tx(UART_COM, "4");
        }
        
        if (button_pressed(BUTTON_E5) == 1) {
          uart_tx(UART_COM, "5");
        }
        
        if (button_pressed(BUTTON_E6) == 1) {
          uart_tx(UART_COM, "6");
        }
        
        if (button_pressed(BUTTON_E7) == 1) {
          uart_tx(UART_COM, "7");
        }
        
        if (button_pressed(BUTTON_E8) == 1) {
          uart_tx(UART_COM, "8");
        }
        
        if (button_pressed(BUTTON_E9) == 1) {
          uart_tx(UART_COM, "9");
        }
        

        
        if (button_pressed(BUTTON_E13) == 1) {
          if (mode) {
						uart_tx(UART_COM, "volume1");
					} else {
						uart_tx(UART_COM, "p"); 
					}
        }
        
        if (button_pressed(BUTTON_E14) == 1) {
					if (mode) {
						uart_tx(UART_COM, "volume2");
					} else {
						uart_tx(UART_COM, "s"); 
					}
        }
        
        if (button_pressed(BUTTON_E15) == 1) {
          if (mode) {
						uart_tx(UART_COM, "volume3");
					} else {
						uart_tx(UART_COM, "r"); 
					}
        }
        
				if (button_pressed(BUTTON_D8) == 1) {
          if (mode) {
						uart_tx(UART_COM, "volume0");
					} else {
						uart_tx(UART_COM, "t"); 
					} 
        }
				
        if (button_pressed(BUTTON_D9) == 1) {
          if (mode) {
						uart_tx(UART_COM, "set_hour");
					} else {
						uart_tx(UART_COM, "u"); 
					} 
        }
        
        if (button_pressed(BUTTON_D10) == 1) {
          if (mode) {
						uart_tx(UART_COM, "set_minute");
					} else {
						uart_tx(UART_COM, "~"); 
					} 
        }
				
				if (button_pressed(BUTTON_E12) == 1) {
          if (mode) {
						uart_tx(UART_COM, "mario_end");
					} else {
						uart_tx(UART_COM, "q"); 
					} 
        }
				
				if (button_pressed(BUTTON_E10) == 1) {
          if (mode) {
						uart_tx(UART_COM, "mario_song");
					} else {
						uart_tx(UART_COM, " "); 
					} 
        }
        
        if (button_pressed(BUTTON_E11) == 1) {
          if (mode) {
						uart_tx(UART_COM, "mario_start");
					} else {
						uart_tx(UART_COM, "\n"); 
					} 
        }


        
				
				//if (button_pressed(BUTTON_D11) == 1) {
					//uart_tx(UART_COM, "D11");
				//}
        

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

