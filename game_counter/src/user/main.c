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
	ticks_init();
	buzzer_init();
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
//        button_update();
//        if (button_pressed(BUTTON_1) == 80) {
//          timer_start(3);
//        } 
//        
//        if (button_released(BUTTON_1) == 1) {
//          timer_start(0);
//        }
//        
//        if (button_pressed(BUTTON_2) == 1) {
//          timer_stop();
//        }
//        
//        if (button_pressed(BUTTON_4) == 1) {
//          if (!is_timer_start()) {
//            // Change mode
//            game_mode = (game_mode + 1) % TIMER_MODE_COUNT;
//            timer_set(game_mode_time[game_mode]);
//            buzzer_control_note(2, 80, game_mode == 0 ? NOTE_D : \
//              (game_mode == 1 ? NOTE_E : (game_mode == 2 ? NOTE_Fs : NOTE_G)), 7);
//          }
//          
//        }


        uart_update();
        timer_update();
        
        
      }
    }
  }
	
}

