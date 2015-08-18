
#include "ticks.h"
static __IO uint32_t sysTickDownCounter;
static __IO uint32_t sysTickUpCounter;

void SysTick_Init(void) {
	/****************************************
	 *SystemFrequency/1000      1ms         *
	 *SystemFrequency/100000    10us        *
	 *SystemFrequency/1000000   1us         *
	 *****************************************/
	while (SysTick_Config(SystemCoreClock / 1000000) != 0) {
	} // One SysTick interrupt now equals 1us
 
}
 
/**
 * This method needs to be called in the SysTick_Handler
 */
void TimeTick_Decrement(void) {
	if (sysTickDownCounter != 0x00) {
		sysTickDownCounter--;
	}
}


void TimeTick_Increment(void) {          //1000us = 1ms ; 1000 ms = 1s 
		if(sysTickUpCounter<10000000)	{   //10s
		sysTickUpCounter++;
	}else sysTickUpCounter =0;

}


u32 get_us_ticks(void){ 		//for getting us ticks				 //1000us = 1ms ; 1000 ms = 1s 
return sysTickUpCounter;					//at most 10000000   10s
}

u32 get_ms_ticks(void){    //for getting ms ticks
return (sysTickUpCounter/1000);
}


void delay_nus(u32 n) {
	sysTickDownCounter = n;
	while (sysTickDownCounter != 0) {
	}
}
 
void delay_1ms(void) {					
	sysTickDownCounter = 1000;
	while (sysTickDownCounter != 0) {
	}
}
 
void delay_nms(u32 n) {
	while (n--) {
		delay_1ms();
	}
}

