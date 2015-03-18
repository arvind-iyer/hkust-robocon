#include "interface.h" 

/**
	* @brief Draw the battery icon on the top right corner of the TFT LCD monitor
	* @param batt: The battery level in voltage times 10 (122 for 12.2V)
	* @retval None
	*/
void draw_battery_icon(u16 batt)
{
	u8 pos = (tft_get_orientation() % 2 ? MAX_HEIGHT : MAX_WIDTH);
	u16 batt_color = 0, batt_boundary = 0;
	u16 batt_w = 0;
	if (batt > BATTERY_USB_LEVEL / 10) {
		//tft_prints(tft_get_max_x_char()-7, 0, "%2d.%d", batt/10, batt%10);
		batt_color = batt <= 114 ? RED : (batt <= 120 ? ORANGE : GREEN);
		batt_boundary = batt <= 114 ? RED : WHITE;
	} else {
		//tft_prints(tft_get_max_x_char()-7, 0, " USB");
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
