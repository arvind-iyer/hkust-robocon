#include "main.h"



u16 ticks_img 	= (u16)-1;
u16 seconds_img = (u16)-1;

void hello(u8 length, u8* data)
{
	buzzer_set_note_period(get_note_period(data[2], data[1]));
	buzzer_control(1, 80);
}

int main(void)
{
	ticks_init();
	buzzer_init();
	tft_init(2, BLACK, WHITE, RED);
	//xbc_init(0);
	//xbc_test_program();
	bluetooth_init();
	bluetooth_rx_add_filter(0xA3, 0xFF, hello);
	
	buzzer_play_song(START_UP, 120, 0);
	
	bluetooth_tx("Hello world\r\n");

	
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			if (ticks_img % 20 == 0) {
				tft_clear();
				tft_prints(0,0,"time: %d", get_seconds());
				tft_prints(0,1,"Bluetooth: %d", bluetooth_rx_state());
				tft_prints(0,2,"CRC: %X", CRC_GetCRC()); 
				tft_update();
			}
		}
	}
}


