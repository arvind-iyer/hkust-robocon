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
	tft_init(0, WHITE, BLACK, RED);

	//can_init();
	//can_rx_init();
  
	buzzer_play_song(START_UP, 120, 0);
  
  while (1) {
    if (ticks_img != get_ticks()) {
      ticks_img = get_ticks();
      tft_clear();
      tft_prints(0, 0, "TEST");
      tft_update();
    }
  }
	
}

