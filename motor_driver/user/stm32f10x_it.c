#include <stdio.h>
//#include "main.h"
#include "stm32f10x_it.h"
//#include "debugger.h"
#include "pid_vel.h"
#include "pid_pos.h"
#include "motion.h"
#include "Robocon_CAN.h"

#define VEL_MOVE 1
#define VEL_STOP 2
#define VEL_ZERO 3
#define POS_MOVE 4
#define POS_LOCK 5
#define CALIBRATION 6

#define stop_byte1	0xfe
#define stop_byte2	0xff
#define buf_size	200

#define rx_buf_size 50

u16 cmd = 0;
u8 cmd_flag = 0, state = 0, last_data = 0, data_cnt = 0;

u8 rx_buffer[buf_size];
u8 tx_buffer[buf_size];
u8 tx_buffer_token=0;
u8 rx_data=0;
u8 tx_data=0;
u8 tx_data_index=0;
u8 tx_data_size=0;



u32 ticks = 0; //adjusts the rate of sending debugging msg
u8 start = 0; //enable or disable pid
u8 mode = 0;
float check = 0;
u32 clock=0;

extern u8 enable_bar;
extern u8 enable_bar_memory;

extern float r_count;

extern float pwm_init;

extern u8 cali_done;


float speed;

u32 i;
void SysTick_Handler(void)
{
	read_encoder();
	get_current();	
	vel_n_pos(); 
	increase_encoder();
/*
	if(ticks<5000)
		ticks++;
	else 
	{
		ticks = 0;
		clock++;
	
	}

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
		get_current(); 
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
/*		else if (state == 1)
		{
		    port_index=rx_data;
			state = 2;
			last_data = 0;
			data_cnt = 0;
		}	  */
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
