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
    if (pre_counter_timer && ticks_counter == 0) {
      // pre_counter 
      if (pre_counter_timer <= TIMER_COUNTING_DOWN_BUZZ) {
        buzzer_control_note(1, 500, NOTE_F, 6);
      }
      
      if (!timer_toggle_flag) {
        --pre_counter_timer;
      }
      
      if (pre_counter_timer == 0) {
        timer_toggle_flag = true; 
      }
    }
    
    switch (timer_mode) {
      case DOWN_COUNTING:
        // Subtracting
        if (ticks_counter == 0) {
          if (pre_counter_timer == 0) { 
            // Normal counter
            
            if (timer_toggle_flag) {
              buzzer_control_note(1, 1500, NOTE_F, 7);
            } else {
              --timer;
            }
            
            if (timer == 0) {
                // Times up!
                //buzzer_control(1, 2000);
                buzzer_control_note(1, 2000, NOTE_F, 7);
                timer_on_flag = false;
            } else {
              if (timer <= TIMER_COUNTING_DOWN_BUZZ || timer == 30) {
                buzzer_control_note(1, 500, NOTE_F, 6);
              }
            }
          }
          
          
        } else if (ticks_counter == 500) {
          
        }
      break;
      
      case UP_COUNTING:
        if (ticks_counter == 0) {
          // Normal counter
          if (timer_toggle_flag && pre_counter_timer == 0) {
            buzzer_control_note(1, 1500, NOTE_F, 7);
          } else {
            if (!pre_counter_timer) {
              ++timer;
            }
          } 
        }
      break;
    }

  } else {
    
  }

  if (ticks_counter == 0) {
    game_counter_colon_set(1);
    /** Update the display **/
    if (pre_counter_timer) {
      game_counter_set_digit_id(0, pre_counter_timer);
      game_counter_set_digit_id(1, pre_counter_timer);
      game_counter_set_digit_id(2, pre_counter_timer);
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
      timer_mode = DOWN_COUNTING;
    } else {
      timer_mode = UP_COUNTING;
    }
    timer_on_flag = true;
    ticks_counter = 0;
    pre_counter_timer = pre_counter;
    buzzer_control_note(1, 2000, NOTE_F, 7);
    timer_toggle_flag = true;
  }
}

void timer_stop(void)
{
  buzzer_control_note(1, 1000, NOTE_F, 6);
  pre_counter_timer = 0; 
  timer_on_flag = false;
}

bool is_timer_start(void)
{
  return timer_on_flag;
}


u32 get_timer_ms(void)
{
  return timer * 1000 + ticks_counter;
}
