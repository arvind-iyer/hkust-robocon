#include "interface.h"
#include <string.h>

static u16 ticks_img 	= (u16)-1;

static u8 menu_selected = 0;	// SELECTED ITEM
static u8 menu_count = 0;

static MENU_ITEM menu_list[MENU_LIST_MAX];

/**
	* @brief System start interface display (to be called directly, delay exists)
	* @param duration: the delay time (in ms) of this function
	* @retval None
	*/
void system_start(u16 duration)
{
  const char* title = "Robocon 2015  Min System 2.0";
  
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_ON);
	tft_clear();

	char tmp[CHAR_MAX_X-1] = "";
	tft_clear();
	tft_prints(1, 1, "[HKUST]");
	tft_prints(1, 2, "[Robotics Team]");
	
	strncpy(tmp, title, tft_get_max_x_char()-2);
	tft_prints(1, 3, "%s", tmp);
	if (strlen(title) >= tft_get_max_x_char()-2) {
		strncpy(tmp, &title[tft_get_max_x_char()-2], tft_get_max_x_char()-2);
		tft_prints(1, 4, "%s", tmp);
	}
	
	
	tft_update();
	
	buzzer_play_song(START_UP, 120, 0);

	u16 prev_text_color = tft_get_text_color();	// Temp color change
	
  tft_prints(0, 7, " Level: %d.%02dV", get_voltage() / 100, get_voltage() % 100);
  tft_prints(0, 8, " Temp: %d.%d%cC", get_temperature() / 10, get_temperature() % 10, 248);
	switch(battery_check()) {
		case BATTERY_USB:
		case BATTERY_OKAY:
			// DO NOTHING
			buzzer_play_song(START_UP, 120, 0);
			tft_set_text_color(DARK_GREEN);
			tft_prints(0, 6, " Battery OK");
		break;
		case BATTERY_LOW:
			buzzer_set_note_period(get_note_period(NOTE_E, 7));
			tft_set_text_color(ORANGE);
			buzzer_control(5, 100);
			tft_prints(0, 6, " LOW BATTERY!");
		break;
		case BATTERY_SUPER_LOW:
			buzzer_set_note_period(get_note_period(NOTE_C, 7));
			tft_set_text_color(RED);
			buzzer_control(10, 50);
			tft_prints(0, 6, " NO BATTERY!");
			tft_update();
			while(1) {
				_delay_ms(200);
				led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_OFF);
				tft_clear_line(7);
				tft_update();
				_delay_ms(200);
				led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_ON); 
				tft_prints(0, 6, " NO BATTERY!");
				tft_update();
			}
		
	}
	tft_update();
	// Reset bg and text color
	tft_set_text_color(prev_text_color);
	

  
  for (u16 i = 0; i < duration / 4; ++i) {
    _delay_ms(4);
    button_update();
    if (BUTTON_ENTER_LISTENER()) {
      break; 
    }
  }
  
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_OFF);
	
}


/**
	* @brief Regular battery check (to be called every 10 seconds)
	* @param None
	* @retval None.
	* @warning Low battery will NOT go into while loop. 
	*/
void battery_regular_check(void)
{
	switch(battery_check()) {
		case BATTERY_USB:
		case BATTERY_OKAY:
			// DO NOTHING
		break;
		case BATTERY_LOW:
			buzzer_set_note_period(get_note_period(NOTE_E, 7));
			buzzer_control(2, 100);
		break;
		case BATTERY_SUPER_LOW:
			buzzer_set_note_period(get_note_period(NOTE_E, 7));
			buzzer_control(6, 200);
		break;
	}
		
}

/**
	* @brief Draw the battery icon on the top right corner of the TFT LCD monitor
	* @param batt: The battery level in voltage times 10 (122 for 12.2V)
	* @retval None
	*/
void draw_battery_icon(u16 batt)
{
  
	u8 pos = (tft_get_max_x_char() - 1) * CHAR_WIDTH;
	u16 batt_color = 0, batt_boundary = 0;
	u16 batt_w = 0;
	if (batt > BATTERY_USB_LEVEL / 10) {
		tft_prints(tft_get_max_x_char()-8, 0, "%2d.%d", batt/10, batt%10);
		batt_color = batt <= 114 ? RED : (batt <= 120 ? ORANGE : GREEN);
		batt_boundary = batt <= 114 ? RED : WHITE;
	} else {
		tft_prints(tft_get_max_x_char()-8, 0, " USB");
		batt_color = SKY_BLUE;
		batt_boundary = WHITE;
		batt = 126;
	}
	
	/* Convert battery level (110 to 126) to the pixel range (0 to 13) */ 
	batt_w = (batt > 126 ? 13 : batt < 110 ? 0 : (batt-110)*13/16);
	for (u8 i = 0; i < 13; i++) {
		for (u8 j = 0; j < 6; j++) {
			tft_put_pixel(pos-19+i, 5+j, i < batt_w ? batt_color : DARK_GREY);
		}
	}
	
	// Top and bottom line
	for (u8 i = 0; i < 17; i++) {
		tft_put_pixel(pos-21+i, 3, batt_boundary);
		tft_put_pixel(pos-21+i, 12, batt_boundary);
	}
	
	// Left and right line
	for (u8 i = 0; i < 8; i++) {
		tft_put_pixel(pos-21, 4+i, batt_boundary);
		tft_put_pixel(pos-5, 4+i, batt_boundary);
	}
	// The right extra lines
	for (u8 i = 0; i < 6; i++) {
		tft_put_pixel(pos-4, 5+i, batt_boundary);
		tft_put_pixel(pos-3, 5+i, batt_boundary);
	}
  
}

/**
	* @brief Draw the top bar (NO tft_clear and tft_update involved)
	* @param None
	* @retval None
	*/
void draw_top_bar(void)
{
	u16 prev_bg_color = tft_get_bg_color();
	u16 prev_text_color = tft_get_text_color();
	// Top bar - time
	tft_set_bg_color(PURPLE);
	tft_set_text_color(WHITE);
	tft_clear_line(0);
	tft_prints(0,0," %02d %02d", get_seconds() / 60, get_seconds() % 60);
	tft_set_text_color(RGB888TO565((500 - Abs(500 - get_ticks())) * 0xFF / 500 * 0x010101));
	tft_prints(3,0,":");
	tft_set_text_color(WHITE);
	
	
	// Top bar - battery (top-right)
	u8 pos = (tft_get_orientation() % 2 ? MAX_HEIGHT : MAX_WIDTH);
	static u8 adc_val_update = 0;
	static u16 temp_voltage;
	static u16 temp_battery;
	
	if (adc_val_update % 100 == 0) {
		temp_voltage = get_voltage();
		temp_battery = get_temperature();
	}
	
  if (get_seconds() % 10 < 5) {
		
    draw_battery_icon(temp_voltage/10);
  } else {
    tft_prints(tft_get_max_x_char() - 7, 0, "%2d.%d%cC%c", temp_battery / 10, temp_battery % 10, 248, 10);
  }
	tft_set_bg_color(prev_bg_color);
	tft_set_text_color(prev_text_color); 	
	++adc_val_update;
}

/**
	* @brief Display the menu (to be called directly, while-loop in side)
	* @param default_id: Default selected / entered menu ID
  * @param pre_enter: True for enterring the selected menu item
  * @retval None
	*/
void menu(u8 default_id, bool pre_enter)
{
	menu_selected = default_id;
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 200 == 1) {
				if (get_seconds() % 10 == 4 && ticks_img == 1) {
					battery_regular_check();
				}
			}

			if (ticks_img % 50 == 2) {
				// Check button input
				button_update();
							
				/** Menu item shift **/
				if (BUTTON_DOWN_LISTENER()) {
					// Go down the list
					if (menu_selected < menu_count - 1) {
						++menu_selected; 
					} else {
						menu_selected = 0;
					}
				} else if (BUTTON_UP_LISTENER()) {
					if (menu_selected > 0) {
						--menu_selected;
					} else {
						menu_selected = menu_count - 1;
					}
				}
				
				/** Enter menu **/
				if (BUTTON_ENTER_LISTENER() || pre_enter) {
					if (menu_list[menu_selected].fx == 0) {
						// NULL FUNCTION
						buzzer_play_song(FAIL_SOUND, 120, 100);
					} else {
						u16 prev_bg_color = tft_get_bg_color();
						u16 prev_text_color = tft_get_text_color();
						tft_clear();
						CLICK_MUSIC;
						menu_list[menu_selected].fx();
						CLICK_MUSIC;
						tft_clear();
						tft_set_bg_color(prev_bg_color);
						tft_set_text_color(prev_text_color);
						pre_enter = false; 
					}
				}        
				/** Change screen orientation **/
				if (button_pressed(BUTTON_1) == 1 || button_pressed(BUTTON_XBC_BACK) == 1) {
					tft_set_orientation((tft_get_orientation() + 1) % 4);
					SUCCESSFUL_MUSIC;
				}
				if (button_pressed(BUTTON_2) == 1) {
					tft_set_orientation((tft_get_orientation() + 3) % 4);
					SUCCESSFUL_MUSIC;
				}
			}	
      
			if (ticks_img % 50 == 7) {
				u16 prev_bg_color = tft_get_bg_color();
				u16 prev_text_color = tft_get_text_color();
				
				tft_clear();
				
				draw_top_bar();
				
				// Menu list
				const u8 items_per_page = tft_get_max_y_char() - 2;
				u8 page_count = menu_count ? (menu_count - 1) / items_per_page : 0; // Start from 0
				u8 current_page = menu_selected / items_per_page;	// Start from 0
				
				for (u8 i = 0 + current_page * items_per_page, y = 0; i < menu_count && y < items_per_page; ++i, ++y) {
					if (i == menu_selected) {
						// Highlight the selected item
						tft_set_bg_color(ticks_img < 500 ? BLACK : DARK_GREY);
						tft_clear_line(y+1);
						tft_set_text_color(WHITE);
					} else {
						tft_set_bg_color(WHITE);
						tft_clear_line(y+1);
						tft_set_text_color(BLACK);
					}
					tft_prints(1, y+1, "%s", menu_list[i].title);
				}
				
				// Bottom bar - page number
				tft_set_text_color(WHITE);
				tft_set_bg_color(PURPLE);
				tft_clear_line(tft_get_max_y_char()-1);
				tft_prints(1, tft_get_max_y_char()-1, "%d/%d", current_page + 1, page_count + 1);
				
				// Reset bg and text color
				tft_set_bg_color(prev_bg_color);
				tft_set_text_color(prev_text_color);
				tft_update();
   
			}
		}
	}
}

/**
  * @brief Add a menu item
  * @param title: Title string
  * @param fx: Function pointer to be called which the item is entered
  * @retval None
*/
void menu_add(const char* title, void (*fx)(void))
{
	if (menu_count < MENU_LIST_MAX) {
		strcpy(menu_list[menu_count].title, title);
		menu_list[menu_count].fx = fx;
		++menu_count;
	}
	
}

/**
  * @brief Event listener for the user interface
* @param ui: The TFT_UI pointer
* @param change: 
*/
void tft_ui_listener(TFT_UI* ui, const TFT_UI_EVENT change) 
{
	
	if (change == tft_ui_event_up || change == tft_ui_event_down) {
		// Switch selection
		if (change == tft_ui_event_up) {
			if (ui->selected_item == 0) {
				ui->selected_item = ui->item_count - 1;
			} else {
				--ui->selected_item;
			}
		} else if (change == tft_ui_event_down) {
			if (ui->selected_item == ui->item_count - 1) {
				ui->selected_item = 0;
			} else {
				++ui->selected_item;
			}
		}
	} else {
	
		TFT_UI_ITEM* item = ui->item_list[ui->selected_item];
		switch (item->type) {
			case tft_ui_checkbox:
				if (tft_ui_event_select) {
					item->ui_item.checkbox.checked ^= 1;
					CLICK_MUSIC;
				}
			break;
			
			case tft_ui_list:
				{
					if (change == tft_ui_event_left) {
						if (item->ui_item.list.selected_int == item->ui_item.list.range.lower) {
							item->ui_item.list.selected_int = item->ui_item.list.range.upper;
						} else {
							--item->ui_item.list.selected_int;
						}
						CLICK_MUSIC;
					}
					else if (change == tft_ui_event_right) {
						if (item->ui_item.list.selected_int == item->ui_item.list.range.upper) {
							item->ui_item.list.selected_int = item->ui_item.list.range.lower;
						} else {
							++item->ui_item.list.selected_int;
						}
						CLICK_MUSIC;
					} else if (tft_ui_event_select && ui->click_event != NULL) {
            ui->click_event(ui->item_list);
          }
				}
			break;
		}
	}
}

void tft_ui_update(const TFT_UI* ui, const bool toggle) 
{
	for (u8 i = 0; i < ui->item_count; ++i) {
		TFT_UI_ITEM* item = ui->item_list[i]; 
		bool item_toggle = toggle;
		if (ui->selected_item != i) {
			item_toggle = false;
		}
		switch (item->type) {
			case tft_ui_checkbox:  
				{
					// Print checkbox
					char checkbox = CHECKBOX_ASCII; 
					if (!item->ui_item.checkbox.checked) {
						checkbox = item_toggle ? HIGHLIGHTED_CHECKBOX_ASCII : CHECKBOX_ASCII;
					} else {
						checkbox = item_toggle ? HIGHLIGHTED_CHECKED_CHECKBOX_ASCII : CHECKED_CHECKBOX_ASCII;
					}
					tft_prints(item->x, item->y, "%c", checkbox);
				}
			break;
			
			case tft_ui_list:
				// Print the two arrows
				tft_prints(item->x, item->y, "%c", item_toggle ? BLACK_BLOCK_ASCII : '<');
				tft_prints(item->x + 1 + item->ui_item.list.width, item->y, "%c", item_toggle ? BLACK_BLOCK_ASCII : '>');
			
			break;
		}
	}
}

u32 tft_ui_get_val(const TFT_UI_ITEM* const item)
{
	switch (item->type) {
		case tft_ui_checkbox:
		
    return item->ui_item.checkbox.checked;
		
		case tft_ui_list:
		
    return item->ui_item.list.selected_int;
	}
  
  return 0; 
}

u8 return_listener(void)
{
  return BUTTON_RETURN_LISTENER();
}
