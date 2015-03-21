#ifndef __SYSTEM_H
#define __SYSTEM_H
#include "main.h"

//#define BUZZER_PORT GPIOA			
//#define BUZZER_PIN	GPIO_Pin_1
//#define BUZZER_RCC	RCC_APB2Periph_GPIOA

//void buzzer_init(void);	 						//init
//void buzzer_control(u8 count, u8 period);		//unit of period is 10ms

u8 _BV(u8);
u16 _BV16(u8 bit);

void enable_bt_printf(COM_TypeDef COM);

void external_Interrup_Init( u8 channel , GPIO_TypeDef * GPIOTYPE );

#endif	/* __SYSTEM_H */		

