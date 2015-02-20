#include "timer.h"

static bool clock_mode = false;
static TIMER_SET_FLAG timer_set_flag = TIMER_SET_OFF;
static u32 alarm_clock = 0;
static bool alarm_flag = false;     /*!< Flag of alarm is on */
static bool timer_on_flag = false;
static bool timer_toggle_flag = false;

static u16 timer = 0;
static u16 timer_next = 0;
static u16 pre_counter_timer = 0;
static u16 ticks_counter = 0;
static TIMER_MODE timer_mode = DOWN_COUNTING;


void timer_init(void)
{
  clock_mode = false;
  timer_on_flag = false;
  timer = 0;
  pre_counter_timer = 0;
  ticks_counter = 0; 
  timer_mode = DOWN_COUNTING;
  ticks_counter = 0;
  timer_set_flag = TIMER_SET_OFF;
  alarm_clock = 0;
  alarm_flag = false;
  timer_next = 0;
}

void timer_clock_set_flag(TIMER_SET_FLAG flag) 
{
   timer_set_flag = flag; 
}

TIMER_SET_FLAG timer_clock_get_flag(void)
{
  return timer_set_flag;
}

bool get_alarm_flag(void)
{
  return alarm_flag;
}

void set_alarm_flag(bool flag) 
{
  alarm_flag = flag;
}

u32 get_alarm(void)
{
  return alarm_clock;
}

bool timer_clock_set(u32 i) 
{
  u32 current_time = get_current_time();
  u8 hour = (current_time / 3600) % 24;
  u8 minute = (current_time / 60) % 60;
  u8 second = current_time % 60;
  
  u8 alarm_hour = (alarm_clock / 3600) % 24;
  u8 alarm_minute = (alarm_clock / 60) % 60;
  
  if (timer_set_flag == TIMER_SET_HOUR) {
    if (hour >= 24) {return false;}
    hour = i;
    set_current_time(hour * 3600 + minute * 60 + second);
    return true;
  } else if (timer_set_flag == TIMER_SET_MINUTE) {
    if (minute >= 60) {return false;}
    minute = i;
    set_current_time(hour * 3600 + minute * 60 + second);
    return true;
  } else if (timer_set_flag == TIMER_SET_ALARM_HOUR) {
    if (alarm_hour >= 24) {return false;}
    alarm_clock = i * 3600 + alarm_minute * 60;
    return true;
  } else if (timer_set_flag == TIMER_SET_ALARM_MINUTE) {
    if (alarm_minute >= 60) {return false;}
    alarm_clock = alarm_hour * 3600 + i * 60;
    return true;
  } else {
    return false;
  }
}

void timer_clock_mode_toggle(bool flag)
{
  if (clock_mode ^ flag) {
    timer_on_flag = false; 
    timer_set_flag = TIMER_SET_OFF;
    clock_mode = flag;
    buzzer_control_note(2, 300, NOTE_G, 7);
  }
}

u8 get_timer_mode(void)
{
  return clock_mode;
}

void timer_set(u16 t)
{
  if (!timer_on_flag) {
    timer = t; 
    ticks_counter = 0;
    
    if (t >= TIMER_COUNT_LIMIT) {t = TIMER_COUNT_LIMIT - 1;}
    
    if (timer > 0) {
      timer_mode = DOWN_COUNTING;
    } else {
      timer_mode = UP_COUNTING;
    }
    timer_next = 0;
  }
}

void timer_next_set(u16 t) 
{
  if (t >= TIMER_COUNT_LIMIT) {t = TIMER_COUNT_LIMIT - 1;}
  timer_next = t; 
}

/**
  * @brief Updating the timer every 10 ms
  */
void timer_update(void)
{
  if (clock_mode) {
    if (ticks_counter == 0) {
      game_counter_colon_set(1);
      ticks_counter = 0;
      u32 display_time = 0;
      u32 current_time = get_current_time();
      switch (timer_set_flag) {
        case TIMER_SET_OFF:
        case TIMER_SET_HOUR:
        case TIMER_SET_MINUTE:
          display_time = current_time;
        break;
        
        case TIMER_SET_ALARM_HOUR:
        case TIMER_SET_ALARM_MINUTE:
          display_time = alarm_clock;
        break;
      }
     
      
      u8 second = display_time % 60;
      u8 minute = (display_time / 60) % 60;
      u8 hour = (display_time / 3600) % 24;
      hour %= 12; // same for AM and PM
      
      game_counter_set_digit_id(0, hour);
      game_counter_set_digit_id(1, minute / 10);
      game_counter_set_digit_id(2, minute % 10);
      
      if (HOUR_ALARM) {
        if (minute == 0 && second == 0) {
          // No hour alarm from 1 am to 8 am (inclusively)
          if (hour < 1 || hour > 8) {
            buzzer_play_song(MARIO_BEGIN, 80, 0);
          }
        }
      }
      
      // ALARM BUZZ!
      if (alarm_flag && current_time == alarm_clock) {
        buzzer_play_song(MARIO_BEGIN, 80, 0);
      }
      
    } else if (ticks_counter == 500) {
      if (timer_set_flag == TIMER_SET_ALARM_HOUR || timer_set_flag == TIMER_SET_ALARM_MINUTE) {
        // Keep turning on colon for alarm setting
        game_counter_colon_set(1);
      } else {
        game_counter_colon_set(0);
      }
      
      
      switch (timer_set_flag) {
        case TIMER_SET_HOUR:
        case TIMER_SET_ALARM_HOUR:
          game_counter_set_digit_id(0, NO_DIGIT);
        break;
        
        case TIMER_SET_MINUTE:
        case TIMER_SET_ALARM_MINUTE:
          game_counter_set_digit_id(1, NO_DIGIT);
          game_counter_set_digit_id(2, NO_DIGIT);
        break;
        
        default:
        break;
      }
      
    }
    
  } else {
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
                  buzzer_control_note(1, 1500, NOTE_F, 7);
                  if (timer_next) {
                    timer = timer_next;
                    timer_next = 0;
                  } else {
                    timer_on_flag = false;
                  }
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
                // Prevent Overflow
                if (timer >= TIMER_COUNT_LIMIT) {
                  timer = 0;
                }
              }
            } 
          }
        break;
      }

    }

    if (ticks_counter == 0) {
      game_counter_colon_set(1);
      /** Update the display **/
      if (pre_counter_timer) {
        game_counter_set_digit_id(0, pre_counter_timer);
        game_counter_set_digit_id(1, pre_counter_timer);
        game_counter_set_digit_id(2, pre_counter_timer);
      } else {
        if (timer_mode == UP_COUNTING && !timer_on_flag) {
          game_counter_display_up();
        } else {
          game_counter_set_time(timer / 60, timer % 60);
        }
      }
    } else if (ticks_counter == 500) {
      if (!timer_on_flag) {
        game_counter_all_off();
      }
      game_counter_colon_set(0);
    }
    
    
    timer_toggle_flag = false;
  }
  
  ticks_counter = (ticks_counter + TIMER_UPDATE_INTERVAL) % 1000;
}

void timer_start(u8 pre_counter)
{
  if (!timer_on_flag) {
    
    timer_on_flag = true;
    ticks_counter = 0;
    if (timer_mode == UP_COUNTING) {
      timer = 0;
    }
    pre_counter_timer = pre_counter;
    buzzer_control_note(1, 1500, NOTE_F, 7);
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
