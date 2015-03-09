
/* Includes ------------------------------------------------------------------*/
#include "delay.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
u8 using_delay = 0 ;



void _delay_us( u32 nus)
{
	
	u32 temp;
	if( using_delay == 0 ){
		using_delay = 1;
		SysTick->LOAD = 9*nus;
		SysTick->VAL = 0x00;
		SysTick->CTRL = 0x01;
		do
		{
			temp=SysTick->CTRL;
		}while((temp&0x01)&&(!(temp&(1<<16))));
		SysTick->CTRL = 0x00;
		SysTick->VAL = 0x00;
		using_delay = 0;
	}
	else{
		nus = nus / 10;
		while( nus -- ){
			simple_delay10_us();
		}
	}
	
}
void _delay_ms( u16 nms )
{
	u32 temp;
	u16 ms ; 
	if( using_delay == 0 ){
		using_delay = 1;
		while( nms ){
		
		ms = ( nms > 1000 ) ? 1000 : nms;
		
		SysTick->LOAD = 9000*ms;
		SysTick->VAL = 0x00;
		SysTick->CTRL = 0x01;
		do
		{
			temp = SysTick->CTRL;
		}while((temp&0x01)&&(!(temp&(1<<16))));
		SysTick->CTRL=0x00;
		SysTick->VAL=0x00;
		
		nms -= ms;
		}
		using_delay = 0;
	}
	else{
		while( nms -- ){
			simple_delay1_ms();
		}
			
	}
}


__asm void simple_delay10_us(){
	MOV		R0, #115
loop
    SUB     R0, R0, #1
    CMP     R0, #0
    BNE        loop
    BX     LR
}

void simple_delay1_ms(){
	u8 i = 0 ; 
	for( i = 0 ; i < 100 ; i ++ )
		simple_delay10_us();
}
