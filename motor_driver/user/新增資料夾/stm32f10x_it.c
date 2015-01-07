#include <stdio.h>
#include "main.h"
#include "stm32f10x_it.h"
#include "debugger.h"
#include "pid_vel.h"
#include "pid_pos.h"
#include "motion.h"
#include "Robocon_CAN.h"

extern u8 control_state;
extern u8 connect_state;

//u32 blinky_count=0;
u16 cmd = 0;
u8 cmd_flag = 0, state = 0, last_data = 0, data_cnt = 0;

u8 rx_buffer[buf_size];
u8 tx_buffer[buf_size];
u8 tx_buffer_token=0;
u8 rx_data=0;
u8 tx_data=0;
u8 tx_data_index=0;
u8 tx_data_size=0;

int ta = 0;


u32 ticks = 0; //adjusts the rate of sending debugging msg
u8 start = 0; //enable or disable pid
u8 mode = 0;
float check = 0;
u32 clock=0;

extern u8 enable_bar;
extern u8 enable_bar_memory;

extern float r_count;

extern float pwm_init;

u32 test_count=0;
extern u8 cali_done;

void motor_test(void)
{
	test_count++;
	mode = VEL_MOVE;

	switch (test_count%8)
	{
	case 0:	
		r_count=0;
		break;

	case 3:
		r_count=-100;
		break;

	case 5:
		r_count=-500;
		break;

	case 6:
		r_count=-200;
		break;
	}
}
float speed;
void SysTick_Handler(void)
{
 
	if(ticks<5000)
		ticks++;
	else 
	{
		ticks = 0;
		clock++;
	
	}
   
	if (ticks%500==0)
	{
		Device1_TX();	
	
		motion_set_motor(speed,1);
	}

	if (clock%4==1)
		speed=0;
	else if (clock%4==3)
		speed=400;
  

 /*
	if(start == 1)
	{  
		if (enable_bar_memory!=enable_bar)
			clear_record();
						
	    vel_n_pos();


		enable_bar_memory=enable_bar;

	 	if(mode == VEL_MOVE)
		{
			enable_bar=1;	 
			check = 1;
		}
		else if(mode == VEL_STOP)
		{
			enable_bar=2;
			check = 2;
		}
		else if(mode == VEL_ZERO)
		{
		 	enable_bar=0;
			check = 3;
		}

		else if(mode == POS_MOVE)
		{
			enable_bar=3;
			check = 4;		
		}
		
		else if (mode == CALIBRATION)
		{
			enable_bar=6;
			check=5;
		}

	}
	
	else 
	{ 
		read_encoder();	 
		clear_record();
		
		if (pwm_init>=0)
			motion_set_motor(pwm_init,1);
		else
			motion_set_motor(-pwm_init,0);

	}
*/		
} 
							   

void USART3_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART3,USART_IT_RXNE)==SET)
	{	           	           
		rx_data = USART_ReceiveData(USART3) & 0xff;
			
		if (state == 0)		//cmd_state
		{
			cmd = rx_data;
			state = 1;
			last_data = 0;
			data_cnt = 0;
		}

		else if (state == 1)	//data_state
		{
			if (data_cnt < rx_buf_size) rx_buffer[data_cnt ++] = rx_data;
			if (last_data == stop_byte1 && rx_data == stop_byte2)
			{
				cmd_flag = 1;
				data_cnt -= 2;
				state = 0;
			}
			last_data = rx_data;
		}
	}

	
	if(USART_GetFlagStatus(USART3,USART_FLAG_TC)==SET)
	{	
	    if (tx_data_size != 0)
		{						  
	        if(tx_data_index<tx_buffer_token-1)
			{						 
		    	tx_data=tx_buffer[tx_data_index];
				tx_data_index++;
				if (tx_data_index>buf_size-1) tx_data_index-=buf_size;
	        	USART_SendData(USART3, tx_data);
    	  
			}
			else
			{
				USART_ITConfig(USART3, USART_IT_TC, DISABLE);
				tx_data=tx_buffer[tx_data_index];
				tx_data_index++;
				if (tx_data_index>buf_size-1) tx_data_index-=buf_size;
	        	USART_SendData(USART3, tx_data);
				tx_data_size=0;

			}	
		}
	} 
}