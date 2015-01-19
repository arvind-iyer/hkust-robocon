#ifndef	__INTERFACE_H
#define	__INTERFACE_H

#include "battery.h"
#include "buzzer.h"
#include "buzzer_song.h"
#include "lcd_red.h"
#include "led.h"

void system_start(u16 duration);
void battery_regular_check(void);


#endif /* __INTERFACE_H */
