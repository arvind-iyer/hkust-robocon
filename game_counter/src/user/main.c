#include "main.h"

u16 ticks_img = (u16)-1;
u8 game_mode = 0;

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
	ticks_init();
	buzzer_init();
  //rtc_init();
  //button_init(); 
  game_counter_init();
  uart_com_init();

  timer_set(60);
  //buzzer_control(1, 1000);
  buzzer_play_song(MARIO_BEGIN, 60, 0);
  
  while (1) {
    if (ticks_img != get_ticks()) {
      ticks_img = get_ticks();
      
      if (get_seconds() < 1) 
      {
        if (ticks_img % 100 == 2) {
          // Beginning digit test
          static u8 i = 0;
          if (i < 7) {
            game_counter_tube_set(0, i, 1);
            game_counter_tube_set(1, i, 1);
            game_counter_tube_set(2, i, 1);
          } else if (i == 7) {
            game_counter_colon_set(1);
          } else if (i == 8) {
            game_counter_all_off();
          }
          ++i;
        }
        continue;
      }
      

      
      if (ticks_img % TIMER_UPDATE_INTERVAL == 6) {


        uart_update();
        timer_update();
        
        
      }
    }
  }
	
}

