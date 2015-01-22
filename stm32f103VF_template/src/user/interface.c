#include "interface.h"

static u16 ticks_img 	= (u16)-1;
static u16 seconds_img = (u16)-1;
static u8 menu_selected = 0;	// SELECTED ITEM
static u8 menu_count = 0;

static MENU_ITEM menu_list[MENU_LIST_MAX];

void system_start(const char* title, u16 duration)
{
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_ON);
	tft_clear();

	char tmp[CHAR_MAX_X-2] = "";
	tft_clear();
	tft_prints(1, 1, "[HKUST]");
	tft_prints(1, 2, "[Robotics Team]");
	
	strncpy(tmp, title, tft_width-2);
	tft_prints(1, 4, "%s", tmp);
	if (strlen(title) >= tft_width-2) {
		strncpy(tmp, &title[tft_width-2], tft_width-2);
		tft_prints(1, 5, "%s", tmp);
	}
	
	
	tft_update();
	
	buzzer_play_song(START_UP, 120, 0);
	
	// Start-up battery check
	battery_adc_update();

	u16 prev_text_color = tft_get_text_color();	// Temp color change
	
	switch(battery_check()) {
		case BATTERY_USB:
		case BATTERY_OKAY:
			// DO NOTHING
			buzzer_play_song(START_UP, 120, 0);
			tft_set_text_color(DARK_GREEN);
			tft_prints(0, 7, " Battery OK");
			tft_prints(0, 8, " Level: %s", get_voltage_string());
		break;
		case BATTERY_LOW:
			buzzer_set_note_period(get_note_period(NOTE_E, 7));
			tft_set_text_color(ORANGE);
			buzzer_control(5, 100);
			tft_prints(0, 7, " LOW BATTERY!");
			tft_prints(0, 8, " Level: %s", get_voltage_string());
		break;
		case BATTERY_SUPER_LOW:
			buzzer_set_note_period(get_note_period(NOTE_C, 7));
			tft_set_text_color(RED);
			buzzer_control(10, 50);
			tft_prints(0, 7, " NO BATTERY!");
			tft_prints(0, 8, " Level: %s", get_voltage_string());
			tft_update();
			while(1) {
				_delay_ms(200);
				led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_OFF);
				tft_clear_line(7);
				tft_update();
				_delay_ms(200);
				led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_ON); 
				tft_prints(0, 7, " NO BATTERY!");
				tft_update();
			}
		
	}
	tft_update();
	// Reset bg and text color
	tft_set_text_color(prev_text_color);
	
	_delay_ms(duration);
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_OFF);
	
}


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

static void draw_battery_icon(u8 batt)
{
	u8 pos = (tft_get_orientation() % 2 ? MAX_HEIGHT : MAX_WIDTH);
	u16 batt_color = 0, batt_boundary = 0;
	u16 batt_w = 0;
	if (batt > BATTERY_USB_LEVEL / 10) {
		tft_prints(tft_width-7, 0, "%2d.%d", batt/10, batt%10);
		batt_color = batt <= 114 ? RED : (batt <= 120 ? ORANGE : GREEN);
		batt_boundary = batt <= 114 ? RED : WHITE;
	} else {
		tft_prints(tft_width-7, 0, " USB");
		batt_color = SKY_BLUE;
		batt_boundary = WHITE;
		batt = 126;
	}
	
	pos = (tft_get_orientation() % 2 ? MAX_HEIGHT : MAX_WIDTH);
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

void draw_top_bar(void)
{
	u16 prev_bg_color = tft_get_bg_color();
	u16 prev_text_color = tft_get_text_color();
	// Top bar - time
	tft_set_bg_color(BLUE2);
	tft_set_text_color(WHITE);
	tft_clear_line(0);
	tft_prints(0,0," %02d %02d", get_seconds() / 60, get_seconds() % 60);
	tft_set_text_color(RGB888TO565((500 - Abs(500 - get_ticks())) * 0xFF / 500 * 0x010101));
	tft_prints(3,0,":");
	tft_set_text_color(WHITE);
	
	
	// Top bar - battery (top-right)
	u8 pos = (tft_get_orientation() % 2 ? MAX_HEIGHT : MAX_WIDTH);
	draw_battery_icon(get_voltage_avg()/10);

	tft_set_bg_color(prev_bg_color);
	tft_set_text_color(prev_text_color); 	
}

void menu(u8 default_id)
{
	menu_selected = default_id;
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 200 == 0) {
				battery_adc_update();
				if (get_seconds() % 2 == 0) {
					battery_regular_check();
				}
			}

			
			if (ticks_img % 50 == 2) {
				// Check button input
				button_update();
				/** Menu item shift **/
				if (button_pressed(BUTTON_JS2_DOWN) == 1 || button_hold(BUTTON_JS2_DOWN, 15, 3)) {
					// Go down the list
					if (menu_selected < menu_count - 1) {
						++menu_selected; 
					} else {
						FAIL_MUSIC;
					}
				} else if (button_pressed(BUTTON_JS2_UP) == 1 || button_hold(BUTTON_JS2_UP, 15, 3)) {
					if (menu_selected > 0) {
						--menu_selected;
					} else {
						FAIL_MUSIC;
					}
				}
				
				/** Change screen orientation **/
				if (button_pressed(BUTTON_1) == 10) {
					tft_set_orientation((tft_get_orientation() + 1) % 4);
					SUCCESSFUL_MUSIC;
				}
				if (button_pressed(BUTTON_2) == 10) {
					tft_set_orientation((tft_get_orientation() + 3) % 4);
					SUCCESSFUL_MUSIC;
				}
				
				/** Enter menu **/
				if (button_pressed(BUTTON_JS2_CENTER) == 1) {
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
					}
				}
			}
			
			if (ticks_img % 50 == 7) {
				u16 prev_bg_color = tft_get_bg_color();
				u16 prev_text_color = tft_get_text_color();
				
				tft_clear();
				
				draw_top_bar();
				
				// Menu list
				const u8 items_per_page = tft_height - 2;
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
				tft_set_bg_color(BLUE2);
				tft_clear_line(tft_height-1);
				tft_prints(1, tft_height-1, "%d/%d", current_page + 1, page_count + 1);
				
				// Reset bg and text color
				tft_set_bg_color(prev_bg_color);
				tft_set_text_color(prev_text_color);
				tft_update();
			}
		}
	}
}

void menu_add(const char* title, void (*fx))
{
	if (menu_count < MENU_LIST_MAX) {
		strcpy(menu_list[menu_count].title, title);
		menu_list[menu_count].fx = fx;
		++menu_count;
	}
	
}

