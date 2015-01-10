#include "main.h"


#define EDROID 1
#define MCS 0
#define REAL 2

//communication
extern u16 cmd;
extern u8 cmd_flag, data_cnt;
extern u8 rx_buffer[];

//admin
extern u8 start; //1 or 0
float conv_start;
extern u8 mode;	 //1, 2, 3, 4, 5
float conv_mode;

//velocity mode
extern float r_count;
extern float d_count;
extern float kp1; //pid for moving
extern float ki1;
extern float kd1;
extern float kp2; //pid for break off
extern float ki2;
extern float kd2;
extern float ip;
extern float ii;
extern float id;
extern float pwm;
extern float pwm_init;
extern s8 dir;
//extern float max_count_vel; //-1 means not set

//position mode
extern float kp_pos;
extern float ki_pos;
extern float kd_pos; 
extern float count_dest;
extern float count_total;
extern float max_acc;	   //max acceleration

extern float max_pwm;    //max speed

//0 means mcs debugger, 1 means edroid debugger, 2 means simulator or real use
u8 debug_mode = REAL;

extern u8 port_index;
extern u8 enable_bar;

//testing
u8 test[4];
extern u8 tx_data_size;
extern u8 tx_buffer[];
extern u8 tx_buffer_token;
extern float delta;
extern float histogram[];
extern float ADC_offset[];
extern u8 cali_done;
extern float d_cal;
extern u8 reach_token;
extern vu16  ADC_ConvertedValue[];
u32 delay_i=0;
u16 ticks_img = 0;

static u8 i = 0;
float integrated_dcount = 0;


int main(void)
{
	system_init(5000);
	system_para_init();
	system_pwm_enable();
	init_memory();
	ticks_init();
	CAN_Configuration();
  
	uart_init(COM1, 115200);
	
	while ((ADC_ConvertedValue[0]<1400 || ADC_ConvertedValue[0]>1700)
	||(ADC_ConvertedValue[1]<1400 || ADC_ConvertedValue[1]>1700)
	|| 1){
		
		
		if (ticks_img != get_ticks()) 
		{
			ticks_img = get_ticks();

			GPIO_WriteBit(GPIOC,GPIO_Pin_0,(ticks_img % 250) < (ADC_ConvertedValue[0]/10));
			GPIO_WriteBit(GPIOC,GPIO_Pin_1,(ticks_img % 250) < (ADC_ConvertedValue[1]/10));

			
			if(ticks_img % 100 == 0) {
				Send_Encoder((s32)(d_cal));
			}


			if(ticks_img % 500 == 0) {
				uart_tx(COM1, "ADC0:%u\r\nADC1:%u\r\n\r\n", ADC_ConvertedValue[0], ADC_ConvertedValue[1]);
			}
		 
		}



	}

	for(delay_i = 0; delay_i < 1000; delay_i++);

	cali_user();



	while (!cali_done)
	{
		if (ticks_img != get_ticks()) 
		{
			ticks_img = get_ticks();
		
			if(ticks_img % 5 == 0)
			{
			//	debugger_msg();	
				Device1_TX();
			}
		}
	}

   
				 GPIO_WriteBit(GPIOC,GPIO_Pin_1,0);
				 GPIO_WriteBit(GPIOC,GPIO_Pin_0,0);
	
	motor_zero();
	for(i = 0; i < 10; i++)	{
		Calibration_Done();
	}
//	motor_set_position(280000,300);
	
	while(1)
	{
	  
		if (ticks_img != get_ticks()) 
		{
			ticks_img = get_ticks();
		
			if(ticks_img % 100 == 0)
			{
				Send_Encoder((s32)(d_cal));
			}
		}

	}	  
}

