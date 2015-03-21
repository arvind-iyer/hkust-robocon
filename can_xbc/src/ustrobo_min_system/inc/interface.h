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
#include "tft.h"
#include "led.h"
#include "xbc_mb.h"

#define MENU_LIST_MAX			20

/**
  * @brief A menu item structure
  */
typedef struct {
	char title[20];     /*!< Title string */
	void (*fx)(void);   /*!< Function pointer to be called when the menu item is entered */ 
} MENU_ITEM;

/**
  * @brief Enumerator of tft_ui item
  */
typedef enum {
	tft_ui_checkbox,
	tft_ui_list
} TFT_UI_ITEM_TYPE;

/**
  * @brief TFT_UI checkbox type
  */
typedef struct {
	bool checked;         /*!< True if the checkbox is checked */
} TFT_UI_CHECKBOX;


/**
  * @brief TFT_UI list type
  */
typedef struct {
	const u8 width;       /*!< The horizontal width (character count) of the list */
	const struct {
		s32 lower, upper;   
	} range;              /*!< Range ([lower, upper]) */
	u32 selected_int;     /*!< The selected item integer */
} TFT_UI_LIST;

typedef struct {
	const TFT_UI_ITEM_TYPE type;    /*!< Type of the tft_ui item */
	const u8 x, y;                  /*!< The tft position of the item */
	union {
		TFT_UI_CHECKBOX checkbox;
		TFT_UI_LIST list;
	} ui_item;                      /*!< The item object */
} TFT_UI_ITEM;

typedef struct {
	const u8 item_count;                    /*!< Number of tft_ui item */
	TFT_UI_ITEM** item_list;                /*!< The array of TFT_UI_ITEM pointer */
	u8 selected_item;                       /*!< The selected item id */
  void (*click_event) (TFT_UI_ITEM**);    /*!< The function pointer to be called when */
} TFT_UI;


void system_start(u16 duration);
void battery_regular_check(void);
void draw_top_bar(void);
void menu(u8 default_id, bool pre_enter);
void menu_add(const char* title, void (*fx)(void));

u8 return_listener(void);

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
