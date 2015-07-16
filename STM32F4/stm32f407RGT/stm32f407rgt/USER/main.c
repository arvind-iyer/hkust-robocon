#include "main.h"

static u32 ticks_msimg = (u32)-1;
static u32 ticks_usimg=(u32)-1;
#define bg_color  WHITE
void testing(void){
	
	LED_control(&PA15, LED_OFF);
	
     if(gpio_read_input(up_button)==0){
		
	LED_control(&PA15,LED_ON);
	buzzer_play_song(CLICK_UP, 125, 0);
	
	}
	
	if(gpio_read_input(down_button)==0){
		
	LED_control(&PA15,LED_ON);
		buzzer_play_song(CLICK_DOWN, 125, 0);
	
	}
	
	if(gpio_read_input(left_button)==0){
		
	LED_control(&PA15,LED_ON);
	buzzer_play_song(CLICK_LEFT, 125, 0);
	
		
	}
	
	if(gpio_read_input(right_button)==0){
		
	LED_control(&PA15,LED_ON);
		buzzer_play_song(CLICK_RIGHT, 125, 0);
		
	
tft_set_pixel_pos(2,3);
	}
	
		if(gpio_read_input(small_button)==0){
	
	LED_control(&PA15,LED_ON);
			buzzer_play_song(CLICK_SMALL_BUTTON, 125, 0);
		
		
	}	
		if(gpio_read_input(center_button)==0){
		
	LED_control(&PA15,LED_ON);
			buzzer_play_song(CLICK_CENTER, 125, 0);
	
			
			
	}
}	

u8 minute =0;
s8 second =0;
u8 hour=0;
u16 ms = 0;

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

void draw_center_ball(){
tft_put_pixel(60,100,RED);
}


int a=0;
int b=0;
void draw_picture(){
	
	tft_put_pixel(a,b,bg_color);
if(gpio_read_input(right_button)==0){a=a+1;};
if(gpio_read_input(left_button)==0){a=a-1;};
if(gpio_read_input(down_button)==0){b=b+1;};
if(gpio_read_input(up_button)==0){b=b-1;};
	tft_prints(0,3,"a%d",a);
    tft_prints(0,4,"b%d",b);
if(gpio_read_input(small_button)==0){
tft_put_pixel(a,b,GREEN);}

}
float r;

void try_cal_float()
{
float q =sqrt(a);
	 tft_prints(0,5,"q%f",q);
	


}

void pic_print(){  //FOR PRING PICTURE
do_picture();	   //print x,y,color for each picture
}

int main(void)
{	//putting global flag
	do_picture_flag=1; //when flag is 1 will do print pic

	
	
	
	
	SysTick_Init();  // must init to make the  interrupt in ticks.h per 1 us
	
	
	
	//initialization of button
	

	button_init();

	//initialization of LED
	LEd_init();
	
	
	//initialization of buzzer
	buzzer_init();
	
	uart_init(115200);
	
	//initialization of Systick

	tft_init( 2,bg_color, BLACK, BLACK);
	
	//LCD init
//	LCD_Init();
	
	//tft_force_clear();

	
	buzzer_play_song(START_UP, 125, 0);
	picture_put_pixel();//for putting data into array in do picture

while (1)  {
	if(ticks_usimg != get_us_ticks()){
	ticks_usimg=get_us_ticks();
		if(ticks_usimg%1==0){
		pic_print();
		}
	
	}


	if (ticks_msimg != get_ms_ticks()) {
			ticks_msimg = get_ms_ticks();  //maximum 1000000
		
			
		if(ticks_msimg%1==0){					//check every 1ms
		    testing();
			buzzer_check();
			system_time(); //every 1ms
	}
		if(ticks_msimg%50==3){  //for processing monitor data
		
			
				//tft_update();
		    tft_clear();
			show_time();
			draw_picture();
			//draw_center_ball();
			try_cal_float();
		
		//	tft_clear();
		//tft_fill_color(GREEN);
			tft_update();			
		}
		
		if(ticks_msimg%1000==2){
			
		//	tft_prints(0,6,"temp");
		//	Get_Temprate();
		//	tft_update();
		
		
		
		}
		
	
		
		}

		
		}
	

		}	
		

	

	



