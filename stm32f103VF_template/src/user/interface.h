#ifndef	__INTERFACE_H
#define	__INTERFACE_H

#include "approx_math.h"
#include "battery.h"
#include "button.h"
#include "buzzer.h"
#include "buzzer_song.h"
#include "ticks.h"
#include "lcd_red.h"
#include "led.h"

#define MENU_LIST_MAX			20
typedef struct {
	char title[20];
	void (*fx)(void);
} MENU_ITEM;

void system_start(const char* title, u16 duration);
void battery_regular_check(void);
void menu(u8 default_id);
void menu_add(const char* title, void (*fx));

#endif /* __INTERFACE_H */
