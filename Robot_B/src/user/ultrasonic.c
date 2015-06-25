#include "ultrasonic.h"

void ultrasonic_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Ultrasonic;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_WriteBit(GPIOE, GPIO_Pin_Ultrasonic, Bit_SET);
}
