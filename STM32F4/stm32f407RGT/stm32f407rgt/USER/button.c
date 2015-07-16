#include "button.h"


void button_init(){
	
	BUTTON_init(small_button);	 //small button
	BUTTON_init(up_button);  //up button
	BUTTON_init(center_button);	 //center button
	BUTTON_init(left_button);	 //left button
	BUTTON_init(right_button);	 //right button
	BUTTON_init(down_button);	 //down button

}

u8 button_read(const GPIO* gpio)
{
	return gpio_read_input(gpio);
}
