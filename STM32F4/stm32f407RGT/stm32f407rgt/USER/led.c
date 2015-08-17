#include "led.h"



void LED_init(const GPIO *gpio){
	GPIO_InitTypeDef  GPIO_InitStructure;
	gpio_rcc_init(gpio);
	GPIO_InitStructure.GPIO_Pin = gpio->gpio_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(gpio->gpio, &GPIO_InitStructure);
	
	if(gpio == &PA15){
		
	GPIO_PinAFConfig(gpio->gpio,gpio->gpio_pin,GPIO_AF_SWJ);
	}
	
}

void LED_control(const GPIO *gpio, u8 state){
	
	if (state == 1){
	GPIO_SetBits(gpio->gpio, gpio->gpio_pin);
		
	}
	if (state == 0){
	GPIO_ResetBits(gpio->gpio, gpio->gpio_pin);
	}

}
u8 state=0;

void LED_blink(const GPIO *gpio)
{

	if (state == 1){
	GPIO_SetBits(gpio->gpio, gpio->gpio_pin);
		state=0;
	}
	else if (state == 0){
	GPIO_ResetBits(gpio->gpio, gpio->gpio_pin);
		state=1;
	}



}	
