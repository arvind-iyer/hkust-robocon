<<<<<<< HEAD
#include "main.h"

u16 ticks_img 	= (u16)-1;
u16 seconds_img = (u16)-1;

int main(void)
{
	ticks_init();
	tft_init(2, BLACK, WHITE, RED);
	xbc_init(0);
	xbc_test_program();
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			if (ticks_img % 20 == 0) {
				tft_clear();
				tft_prints(0,0,"time: %d", get_seconds());
				tft_prints(0,1,"Test finsh");
				tft_update();
			}
		}
	}
}
=======
#include "main.h"

u16 ticks_img 	= (u16)-1;
u16 seconds_img = (u16)-1;

int main(void)
{
	ticks_init();
	adc_init();
	
	//tft_enable();
	
	uart_init(COM1, 115200);
	
	uart_printf_enable(COM1);
	
	
	tft_init(1,WHITE,BLACK,GREEN);
	
	while(1) {
			if(get_ticks() % 50 == 0)
			{
				tft_clear();
				tft_prints(0,0,"Voltage: %d", get_voltage());
				tft_prints(0,1,"TEMP: %d", get_temp());
				tft_update();
				
			}		
			if(get_ticks() %500 == 0)
			{
				printf("Voltage: %d", get_voltage());
				printf("TEMP: %d", get_temp());
			}
				
		}

}


>>>>>>> FETCH_HEAD
