#include "timer.h"

static bool clock_mode = false;
static u32 timer_off_idle_ms = 0;
static TIMER_SET_FLAG timer_set_flag = TIMER_SET_OFF;
static ALARM alarms[ALARM_COUNT] = {{0,false,BIRTHDAY_SONG}};
static u8 alarm_id_tmp = 0;
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
  for (u8 i = 0; i < ALARM_COUNT; ++i) {
    alarms[i].time = 0;
    alarms[i].flag = false;
    alarms[i].music = BIRTHDAY_SONG; 
  }
  timer_next = 0;
  timer_off_idle_ms = 0;
}

void timer_reset_idle(void)
{
	timer_off_idle_ms = 0;
}

void timer_clock_set_flag(TIMER_SET_FLAG flag) 
{
   timer_set_flag = flag; 
}

TIMER_SET_FLAG timer_clock_get_flag(void)
{
  return timer_set_flag;
}


const ALARM* get_alarm(u8 i)
{
  return &alarms[i];
}

void set_alarm_flag(u8 i, bool flag)
{
  alarms[i].flag = flag;
}

void alarm_id_tmp_set(u8 i)
{
  alarm_id_tmp = i;
}

bool timer_clock_set(u32 i) 
{
  u32 current_time = get_current_time();
  u8 hour = (current_time / 3600) % 24;
  u8 minute = (current_time / 60) % 60;
  u8 second = current_time % 60;
  
  u8 alarm_hour = (alarms[alarm_id_tmp].time / 3600) % 24;
  u8 alarm_minute = (alarms[alarm_id_tmp].time / 60) % 60;
  
  if (timer_set_flag == TIMER_SET_HOUR) {
    if (i >= 24) {return false;}
    hour = i;
    set_current_time(hour * 3600 + minute * 60 + second);
    return true;
  } else if (timer_set_flag == TIMER_SET_MINUTE) {
    if (i >= 60) {return false;}
    minute = i;
    set_current_time(hour * 3600 + minute * 60 + second);
    return true;
  } else if (timer_set_flag == TIMER_SET_ALARM_HOUR) {
    if (i >= 24) {return false;}
    alarms[alarm_id_tmp].time = i * 3600 + alarm_minute * 60;
    return true;
  } else if (timer_set_flag == TIMER_SET_ALARM_MINUTE) {
    if (i >= 60) {return false;}
    alarms[alarm_id_tmp].time = alarm_hour * 3600 + i * 60;
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
    timer_off_idle_ms = 0;
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
	timer_reset_idle();
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
          display_time = alarms[alarm_id_tmp].time;
        break;
      }
     
      
      u8 second = display_time % 60;
      u8 minute = (display_time / 60) % 60;
      u8 hour = (display_time / 3600) % 24;
      
      if (HOUR_ALARM) {
        if (current_time % 3600 == 0) {
          // No hour alarm from 1 am to 8 am (inclusively)
          if (hour < 1 || hour > 8) {
            if (hour == 0 || hour == 12) {
              MARIO_MUSIC;
            } else if (hour >= 0 && hour <= 11) {
              MARIO_END_MUSIC;
            } else {
              MARIO_BEGIN_MUSIC;
            }
          }
        }
      }
      
      hour %= 12; // same for AM and PM
      
      game_counter_set_digit_id(0, hour);
      game_counter_set_digit_id(1, minute / 10);
      game_counter_set_digit_id(2, minute % 10);
      

      
      // ALARM BUZZ!
      for (u8 i = 0; i < ALARM_COUNT; ++i) {
        const ALARM* alarm = get_alarm(i);
        if (alarm->flag && current_time == alarm->time) {
          static u8 i = 0;
          i = (i+1) % 2;
          if (i == 0) {
            BIRTHDAY_MUSIC;
          } else {
            MARIO_MUSIC; 
          }
         
        }
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
      timer_off_idle_ms = 0;
      if (pre_counter_timer && ticks_counter == 0) {
        // pre_counter 
        if (pre_counter_timer <= TIMER_COUNTING_DOWN_BUZZ) {
          buzzer_control_note(1, 200, NOTE_F, 6);
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
                if (timer <= TIMER_COUNTING_DOWN_BUZZ) {
                  buzzer_control_note(1, 200, NOTE_F, 6);
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
    
    if (!timer_on_flag) {
      // Timer idle time check
      timer_off_idle_ms += TIMER_UPDATE_INTERVAL;
      if (timer_off_idle_ms >= IDLE_TIME_THRESHOLD) {
        timer_clock_mode_toggle(true);
      }
    }
    
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
