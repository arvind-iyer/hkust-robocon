#include "main.h"

u16 ticks_img 	= (u16)-1;
u16 seconds_img = (u16)-1;

int main(void)
{
	ticks_init();
	buzzer_init();
	adc_init();
	
	//tft_enable();
	
	uart_init(COM1, 115200);
	
	uart_printf_enable(COM1);
	
	
	tft_init(1,YELLOW,RED,GREEN);
	_delay_ms(1000);
	buzzer_play_song(START_UP, 120, 0);
	
	while(1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			if(ticks_img % 50 == 0)
			{
				tft_clear();
				tft_prints(0,0,"Seconds: %d", get_seconds());
				tft_prints(0,1,"Ticks: %d", get_ticks());
				tft_update();
				//printf("%d   ",get_voltage());
			}			
		}

	}
}
