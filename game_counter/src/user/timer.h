#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"
#include <stdbool.h>
#include "usart.h"
#include "game_counter.h"
#include "buzzer.h"
#include "rtc.h"
#include "buzzer_song.h"
#include "ticks.h"

#define TIMER_UPDATE_INTERVAL     10  /** MUST BE A FACTOR OF 500 */
#define TIMER_COUNTING_DOWN_BUZZ  10


#define TIMER_COUNT_LIMIT          (10 * 60) // 10 minutes (exclusive)
#define TIMER_CLOCK_LIMIT          (60 * 60 * 24)
#define HOUR_ALARM                 1

#define ALARM_COUNT               10
#define IDLE_TIME_THRESHOLD       600000 // 600 s / 10 minutes

#define	LOTTERY_TIME							15000		// 15s

typedef struct {
  u8 minute;
  u8 second;
} TIMER;

typedef enum {
  UP_COUNTING,
  DOWN_COUNTING
} TIMER_MODE;

typedef enum {
  TIMER_SET_OFF,
  TIMER_SET_HOUR,
  TIMER_SET_MINUTE,
  TIMER_SET_ALARM_HOUR,
  TIMER_SET_ALARM_MINUTE,
	TIMER_SET_DAYS_LEFT
} TIMER_SET_FLAG;

typedef struct {
  u32 time;
  bool flag;
  const MUSIC_NOTE* music;
} ALARM;

void timer_init(void);
void timer_reset_idle(void);
void timer_clock_set_flag(TIMER_SET_FLAG flag);
TIMER_SET_FLAG timer_clock_get_flag(void);

const ALARM* get_alarm(u8 i);
void set_alarm_flag(u8 i, bool flag);
void alarm_id_tmp_set(u8 i);
  
bool timer_clock_set(u32 i);
void timer_clock_mode_toggle(bool flag);
u8 get_timer_mode(void);
void timer_set(u16 t);
void timer_next_set(u16 t);
void timer_set_next_action(void (*fx)(void));

void timer_update(void);
void timer_start(u8 pre_counter);
void timer_stop(void);
bool is_timer_start(void);
u32 get_timer_ms(void);
void lottery_draw(void);

#endif  /* __TIMER_H */
