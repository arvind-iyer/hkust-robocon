#include "main.h"

int main(void)
{
	adc_init();
	tft_init(2,WHITE,BLACK,RED);
	tft_enable();
	
	uart_init(COM1, 115200);
	ticks_init();
	uart_printf_enable(COM1);
	while(1)
	{
		if(get_ticks() % 50 == 0)
		{
			tft_clear();
			tft_prints(0,4,"VOLTAGE: %d.%d",(u16)get_voltage()/100,get_voltage()%100);
			tft_update();
			//printf("%d   ",get_voltage());
		}
	}
}
