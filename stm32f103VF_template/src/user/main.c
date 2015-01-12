#include "main.h"

int main(void)
{
	adc_init();
	tft_init(1,YELLOW,RED,GREEN);
	tft_enable();
	
	uart_init(COM1, 115200);
	ticks_init();
	uart_printf_enable(COM1);
	while(1)
	{
		if(get_ticks() % 50 == 0)
		{
			tft_clear();
			tft_prints(0,0,"Seconds: %d", get_seconds());
			tft_prints(0,1,"Ticks: %d", get_ticks());
			tft_update();
			//printf("%d   ",get_voltage());
		}
	}
}
