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
