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
	//can_rx_init();
  
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
        
      if (ticks_img % 20 == 2) {
        xbc_loop();
        xbc_tx_data();
      }
      
      if (ticks_img % 20 == 4) {
        tft_clear();
        
        
        tft_prints(0, 0, "Hello world %d!", get_seconds());
        tft_prints(0, 1, "State: %d", usb_get_state());
        tft_prints(usb_get_state(), 2, "%d", usb_get_state());
        tft_prints(0, 3, "%d %d %d", get_xbc_data(0), get_xbc_data(1), get_xbc_data(2));
        tft_update();
        
        
      }

      
      
    }
  }
	
}

