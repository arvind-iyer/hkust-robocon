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



void test(){
	u8 temp;
	u8 humi;
	DHT11_Read_Data(&temp,&humi);
	tft_prints(0,2,"t%d,h%d",temp,humi);
	
	
	usart_rx();
}

int main(void)
{	//putting global flag
	
	SysTick_Init();  // must init to make the  interrupt in ticks.h per 1 us
	
buzzer_init();	 //initialization of buzzer

uart_init(9600);
	ADC1_init();
DHT11_Init();

tft_init( 2,GREEN, BLACK, BLACK);
buzzer_play_song(START_UP, 125, 0);
LED_init(&PA15);//THE ONLY LED ON THE BOARD
LED_control(&PA15,1);
//gpio_init(&PE3, GPIO_Mode_IN, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP);
init_gpio_interrupt(SMALL_BUTTON_GPIO, EXTI_Trigger_Falling);
init_gpio_interrupt(JOY_CENTER_GPIO, EXTI_Trigger_Falling);
int temp =1;
while (1)  {

	if(ticks_usimg != get_us_ticks())   // using us to control  for some important control
	{
		ticks_usimg=get_us_ticks();
		
		
		
		


	if (ticks_msimg != get_ms_ticks()) 
	{
		ticks_msimg = get_ms_ticks();  //maximum 1000000	
			buzzer_check();
			system_time(); //every 1ms

		
		if(ticks_msimg%100==1){LED_blink(&PA15);}
		
		if(ticks_msimg%1000==0)
		{  //for processing monitor data
			tft_clear();
			test();
			tft_prints(0,3,"degree:%f",get_temperature());
			show_time();
			//send_tx();
			//display_button_data();
			tft_update();	
			
			
			//usart_tx('1');
		
		
			usart_print("whatsapp");
		
		}
	}
}	
}	
	

}	

	



