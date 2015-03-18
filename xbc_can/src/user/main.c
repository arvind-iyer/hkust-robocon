#include "main.h"

u16 ticks_img = (u16) -1;
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
	//button_init();
	//led_init();
  usb_init();
  usb_start_run();
	tft_init(0, WHITE, BLACK, RED);
  can_init();
  can_rx_init();  
  xbc_rx_init();
  
	//buzzer_play_song(START_UP, 120, 0);
  buzzer_control_note(1, 300, NOTE_A, 5);
  
  bool connection = false;
  
  while (1) {
    if (ticks_img != get_ticks()) {
      ticks_img = get_ticks();
  
      if (!usb_connected()) {
        // Faster loop if USB not connected
        if (connection) {connection = false;}
        usb_main_loop();
        if (get_seconds() >= 1 && ticks_img == 3) { 
          buzzer_control_note(2, 100, NOTE_Fs, 3);
        }
      } else {
        if (!connection) {
          connection = true;
          buzzer_play_song(CONNECTED, 120, 0);
        }
        
        if (ticks_img % 5 == 0) {
          usb_main_loop(); 
        }
      }
        
      if (ticks_img % 10 == 2) {
        xbc_loop();
        xbc_tx_data();
      }
      
      if (ticks_img % 20 == 4) {
        tft_clear();
        
        u8 xbc_rx_lcd_update_predicate = 0;
        u8 usb_connection = usb_connected();
        
        if (usb_connection) {
          xbc_rx_lcd_update_predicate = xbc_rx_lcd_update();  // This line run first as it prints
        }
        
        if (!usb_connection || !xbc_rx_lcd_update_predicate) { 
          tft_prints(0, 0, "XBOX Controller");
          tft_prints(0, 1, "Time: %d.%02d", get_seconds(), get_ticks() / 100);
          tft_prints(0, 2, "USB state: %d", usb_get_state());
          u32 tmp_time = xbc_get_last_rx_time_ms();
          tft_prints(0, 3, "Last RX: %d.%02d", tmp_time / 1000, (tmp_time % 1000) / 100);
          tft_prints(0, 4, "XBC_DATA (hex)");
          tft_prints(0, 5, " %02X %02X %02X %02X", get_xbc_data(0), get_xbc_data(1), get_xbc_data(2), get_xbc_data(3));
          tft_prints(0, 6, " %02X %02X %02X %02X", get_xbc_data(4), get_xbc_data(5), get_xbc_data(6), get_xbc_data(7));
          tft_prints(0, 7, " %02X %02X %02X %02X", get_xbc_data(8), get_xbc_data(9), get_xbc_data(10), get_xbc_data(11));
          tft_prints(0, 8, " %02X %02X %02X %02X", get_xbc_data(12), get_xbc_data(13), get_xbc_data(14), get_xbc_data(15));
          
        }
        
        tft_update();
      }

      
      
    }
  }
	
}

