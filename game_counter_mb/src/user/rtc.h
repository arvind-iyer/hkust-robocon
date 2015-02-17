#ifndef __RTC_H 
#define __RTC_H

#include "stm32f10x.h" 
#include "stm32f10x_rtc.h" 


#define MAX_RTC_VAL   (60 * 60 * 24)  // 23:59:59 + 1


void rtc_init(void);
void rtc_interrupt_init(void (*fx)(void));
void rtc_set_time(u32 time);
u32 rtc_get_time(void);

#endif  /* __RTC_H */
