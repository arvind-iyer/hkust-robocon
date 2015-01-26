#ifndef	__INTERFACE_H
#define	__INTERFACE_H

#include <stdbool.h>
#include "stm32f10x.h"
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

typedef enum {
	tft_ui_checkbox,
	tft_ui_list
} TFT_UI_ITEM_TYPE;

typedef struct {
	bool checked; 
} TFT_UI_CHECKBOX;

typedef struct {
	const u8 width;
	const struct {
		s32 lower, upper;
	} range; 
	u32 selected_int;
} TFT_UI_LIST;

typedef struct {
	const TFT_UI_ITEM_TYPE type;
	const u8 x, y;
	union {
		TFT_UI_CHECKBOX checkbox;
		TFT_UI_LIST list;
	} ui_item;
} TFT_UI_ITEM;

typedef struct {
	const u8 item_count;
	TFT_UI_ITEM** item_list; 
	u8 selected_item;
} TFT_UI;


void system_start(const char* title, u16 duration);
void battery_regular_check(void);
void draw_top_bar(void);
void menu(u8 default_id, bool pre_enter);
void menu_add(const char* title, void (*fx)(void));

typedef enum {
	tft_ui_event_left,
	tft_ui_event_right,
	tft_ui_event_up,
	tft_ui_event_down,
	tft_ui_event_select
} TFT_UI_EVENT;
void tft_ui_listener(TFT_UI* ui, const TFT_UI_EVENT change) ;
void tft_ui_update(const TFT_UI* ui, bool toggle);
u32 tft_ui_get_val(const TFT_UI_ITEM* item);


#endif /* __INTERFACE_H */
