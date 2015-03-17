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
  
	buzzer_play_song(START_UP, 120, 0);
  
  while (1) {
    if (ticks_img != get_ticks()) {
      ticks_img = get_ticks();
  
      if (!usb_connected()) {
        // Faster loop if USB not connected
        usb_main_loop();
        if (ticks_img == 3) {
          buzzer_control_note(2, 100, NOTE_Fs, 5);
        }
      } else {
        if (ticks_img % 5 == 0) {
          usb_main_loop(); 
        }
      }
        
      if (ticks_img % 20 == 2) {
        xbc_loop();
      }
      
      if (ticks_img % 20 == 4) {
        tft_clear();
        
        
        tft_prints(0, 0, "Hello world %d!", get_seconds());
        tft_prints(0, 1, "State: %d", usb_get_state());
        tft_prints(usb_get_state(), 2, "%d", usb_get_state());
        tft_prints(0, 3, "%d %d %d", get_xbc_data(0), get_xbc_data(1), get_xbc_data(2));
        tft_update();
        
        
      }
      
      if (ticks_img % 900 == 2) {
        xbc_tx_data();
      }
      
      
    }
  }
	
}

