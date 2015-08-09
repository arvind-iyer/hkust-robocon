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
//printf("MCU serving time(h:m:s:ms)  %d : %d : %d : %d \n" ,hour,minute,second,ms);
	tft_prints(0,0,"(h:m:s:ms) ");
	tft_prints(0,1, "%d : %d : %d : %d \n" ,hour,minute,second,ms);
		
}

void draw_ball(int x, int y){
tft_put_pixel(x,y,RED);
}

void display_button_data()
{
	int j = 0;
	for(  j = 0; j < BUTTON_COUNT; j++) 
	{
		tft_prints(0,j+2, "%d  %d", button_pressed(j), button_released(j));
		//tft_prints(0, j+2, "%d", gpio_read_input(buttons[j]));
	}
}

char data[100]={1,2,3,4,5,6,7,8,9,10};
int a =0;
u8 sended =0;

void send_tx(){
	

}

int main(void)
{	//putting global flag
	
	SysTick_Init();  // must init to make the  interrupt in ticks.h per 1 us
	
buzzer_init();	 //initialization of buzzer

uart_init(9600);



////uart_init(115200);

tft_init( 2,GREEN, BLACK, BLACK);
buzzer_play_song(START_UP, 125, 0);
LED_init(&PA15);//THE ONLY LED ON THE BOARD

//gpio_init(&PE3, GPIO_Mode_IN, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP);
init_gpio_interrupt(SMALL_BUTTON_GPIO, EXTI_Trigger_Falling);
init_gpio_interrupt(JOY_CENTER_GPIO, EXTI_Trigger_Falling);
while (1)  {

	if(ticks_usimg != get_us_ticks())
	{
		ticks_usimg=get_us_ticks();




	
	
	
	if (ticks_msimg != get_ms_ticks()) 
	{
		ticks_msimg = get_ms_ticks();  //maximum 1000000	
			buzzer_check();
			system_time(); //every 1ms

		if(ticks_msimg%500==3)
		{  //for processing monitor data
			tft_clear();
		
			show_time();
			//send_tx();
			//display_button_data();
			tft_update();	
			
			
			//usart_tx('1');
		
			//send_tx();
			usart_print("aaa");
			
		}
	}
}	
}	
	

}	

	



