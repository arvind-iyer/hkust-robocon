#include "timer.h"

static bool timer_on_flag = false;
static bool timer_toggle_flag = false;
static u16 timer = 0;
static u16 pre_counter_timer = 0;
static u16 ticks_counter = 0;
static TIMER_MODE timer_mode = DOWN_COUNTING;

void timer_init(void)
{
  timer_on_flag = false;
  timer = 0;
  pre_counter_timer = 0;
  ticks_counter = 0; 
  timer_mode = DOWN_COUNTING;
  ticks_counter = 0;
}

void timer_set(u16 t)
{
  if (t > 9 * 60 + 59) {t = 9 * 60 + 59;}
  if (!timer_on_flag) {
    timer = t; 
    ticks_counter = 0;
  }
}

/**
  * @brief Updating the timer every 10 ms
  */
void timer_update(void)
{
  
  if (timer_on_flag) {
    
    
    switch (timer_mode) {
      case DOWN_COUNTING:
        // Subtracting
        if (ticks_counter == 0) {
          
          if (pre_counter_timer) {
            // pre_counter 
            if (pre_counter_timer <= TIMER_COUNTING_DOWN_BUZZ) {
              buzzer_control(1, 500);
            }
            
            if (!timer_toggle_flag) {
              --pre_counter_timer;
            }
            
            if (pre_counter_timer == 0) {
              timer_toggle_flag = true; 
            }
          }

          if (pre_counter_timer == 0) { 
            // Normal counter
            
            if (timer_toggle_flag) {
              buzzer_control(1, 1500);
            } else {
              --timer;
            }
            
            if (timer == 0) {
                // Times up!
                buzzer_control(1, 2000);
                timer_on_flag = false;
            } else {
              if (timer <= TIMER_COUNTING_DOWN_BUZZ || timer == 30) {
                buzzer_control(1, 500);
              }
            }
          }
          
          
        } else if (ticks_counter == 500) {
          
        }
      break;
      
      case UP_COUNTING:
        
      break;
    }

  } else {
    
  }

  if (ticks_counter == 0) {
    game_counter_colon_set(1);
    /** Update the display **/
    if (pre_counter_timer) {
      game_counter_set_time(pre_counter_timer / 60, pre_counter_timer % 60);
    } else {
      game_counter_set_time(timer / 60, timer % 60);
    }
  } else if (ticks_counter == 500) {
    if (!timer_on_flag) {
      game_counter_all_off();
    }
    game_counter_colon_set(0);
  }
  
  ticks_counter = (ticks_counter + TIMER_UPDATE_INTERVAL) % 1000;
  timer_toggle_flag = false;
}

void timer_start(u8 pre_counter)
{
  if (!timer_on_flag) {
    if (timer > 0) {
      timer_on_flag = true;
      ticks_counter = 0;
      pre_counter_timer = pre_counter;
    } 
    buzzer_control(1, 500);
    timer_toggle_flag = true;
  }
}

void timer_stop(void)
{
  buzzer_control(1, 500);
  timer_on_flag = false;
}
