#include "main.h"

u16 ticks_img = (u16) -1;			// Trival value
u16 seconds_img = (u16) -1;		// Trival value




int main(void)
{
	/*** Your code ***/

	ticks_init();
	buzzer_init();
	led_init();

	
	while(1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img == 0) {
				buzzer_control(3, 100);
				led_control((LED) (LED_D1 | LED_D2 | LED_D3), (LED_STATE) (get_seconds() % 2));
			}
		}
		
		
	}

}

