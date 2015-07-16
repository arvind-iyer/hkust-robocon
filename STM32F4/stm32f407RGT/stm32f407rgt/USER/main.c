#include "main.h"



u8 minute =0;
s8 second =0;
u8 hour=0;
u16 ms = 0;

static u32 ticks_msimg = (u32)-1;
static u32 ticks_usimg=(u32)-1;

void system_time(){
	
	ms = ms +1;
	if (ms ==1000){second=second +1;ms=0;}
	if (second==60){minute=minute+1;second=0;}
	if (minute==60){hour=hour+1;minute=0;}

}

void show_time(){
	//tft_clear();
printf("MCU serving time(h:m:s:ms)  %d : %d : %d : %d \n" ,hour,minute,second,ms);
	tft_prints(0,0,"(h:m:s:ms) ");
	tft_prints(0,1, "%d : %d : %d : %d \n" ,hour,minute,second,ms);
		
}

void draw_ball(int x, int y){
tft_put_pixel(x,y,RED);
}

int main(void)
{	//putting global flag
	
	SysTick_Init();  // must init to make the  interrupt in ticks.h per 1 us
	button_init();	 //initialization of button
	buzzer_init();	 //initialization of buzzer
	uart_init(115200);
	tft_init( 2,WHITE, BLACK, BLACK);
	//delay_nms(1000);
	buzzer_play_song(START_UP, 125, 0);

	while (1)  {
		if(ticks_usimg != get_us_ticks())
		{
			ticks_usimg=get_us_ticks();
		}
		
		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
				buzzer_check();
				system_time(); //every 1ms

			if(ticks_msimg%50==3)
			{  //for processing monitor data
				//tft_update();
				tft_clear();
				show_time();
				tft_update();			
				
			}
			
			
			
			
		}
	}	
}	
		

	

	



