#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"
#include <stdbool.h>
#include "usart.h"
#include "game_counter.h"
#include "buzzer.h"



#define TIMER_UPDATE_INTERVAL     10  /** MUST BE A FACTOR OF 500 */
#define TIMER_COUNTING_DOWN_BUZZ  5

#define TIMER_COUNT_LIMIT          (10 * 60) // 10 minutes (exclusive)
typedef struct {
  u8 minute;
  u8 second;
} TIMER;

typedef enum {
  UP_COUNTING,
  DOWN_COUNTING
} TIMER_MODE;

void timer_init(void);
void timer_set(u16 t);

void timer_update(void);
void timer_start(u8 pre_counter);
void timer_stop(void);
bool is_timer_start(void);
u32 get_timer_ms(void);


#endif  /* __TIMER_H */
