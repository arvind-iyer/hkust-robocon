#include "linear_ccd.h"

void CLK(u8 state){//self make clock

		if (state == 1){
	GPIO_SetBits(CLK_PORT, CLK_PIN);
		state=0;
	}
	else if (state == 0){
	GPIO_ResetBits(CLK_PORT,CLK_PIN);
		state=1;
	}
}


void SI(u8 bit){

	if(bit==1){
GPIO_SetBits(SI_PORT,SI_PIN);}
	

	else
		GPIO_ResetBits(SI_PORT,SI_PIN);
	
	}



float AO(){
	u16 temp=0;
	for(u8 i=0;i<10;++i){
	temp=temp+ADC_raw_data(AO_channel);
	
	}
return temp*0.00026996337;

}


float linear_ccd_buffer[128];

void linear_ccd_read(){
	SI(0);
	CLK(0);
	delay_nus(1);
	SI(1);
for(u8 n=0;n<128;n++)
	{
		delay_nus(1);
		CLK(1);		
		SI(0);
		delay_nus(1);
		linear_ccd_buffer[n]=AO();
		CLK(0);
	}
	
	
  
	tft_prints(0,2,"m:%f",linear_ccd_buffer[64]);//black ,,return value : 4.7 v   white  0.0xxv
	tft_prints(0,3,"l:%f",linear_ccd_buffer[20]);
	tft_prints(0,4,"r:%f",linear_ccd_buffer[126]);
	
	
	for(u8 m=80;m<100;m++)
	{
	for(u8 n=18;n<128;n++)
	{
		if(linear_ccd_buffer[n]<2.5){	
			tft_put_pixel(n,m,BLACK);}
		
			else{tft_put_pixel(n,m,WHITE);}
	}
}

	
	
	
}





void linear_ccd_init()
{//initialization of clk PA1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = CLK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CLK_PORT, &GPIO_InitStructure);
	
//initialization of si PA0
	GPIO_InitStructure.GPIO_Pin = SI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(SI_PORT, &GPIO_InitStructure);
	
}