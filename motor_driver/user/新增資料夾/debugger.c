#include "debugger.h"

//ticks adjusts the rate of sending msg
extern u32 ticks;

//declare all the extern variables to be sent
extern float pwm;
extern u8 start;
u8 _start;
extern u8 mode;
extern u8 dir;
u8 _dir;

extern float d_count;
extern float r_count;
extern u16 cmd;
extern u8 check;

extern u8 connect_state;

extern u8 tx_buffer[];
extern u8 tx_buffer_token;
extern u8 tx_data_size;
float				float32[6];
unsigned char		conversion[4];

//use time division multiplexing
//changeable according to needs

void update_token(void)
{
    tx_buffer_token++;
	
	if (tx_buffer_token>buf_size-1)
	    tx_buffer_token=0;
}

void system_shake_hand (u8 shake)
{
	connect_state = shake;
	tx_buffer[tx_buffer_token]=shake;
	update_token();
	tx_buffer[tx_buffer_token]=stop_byte1;
	update_token();
	tx_buffer[tx_buffer_token]=stop_byte2;
	update_token();
    tx_data_size=3;
	USART_ITConfig(USART3, USART_IT_TC, ENABLE);

}


void send_debug_msg(u8 index, float data)
{
    u8 *tmp = (u8 *) &data;
	tx_buffer[tx_buffer_token]=send_float32_cmd + index;
	update_token();

	tx_buffer[tx_buffer_token]=tmp[0];
	update_token();
	tx_buffer[tx_buffer_token]=tmp[1];
	update_token();
	tx_buffer[tx_buffer_token]=tmp[2];
	update_token();
	tx_buffer[tx_buffer_token]=tmp[3];
	update_token();

	tx_buffer[tx_buffer_token]=stop_byte1;
	update_token();
	tx_buffer[tx_buffer_token]=stop_byte2;
	update_token();

    tx_data_size=7;
	USART_ITConfig(USART3, USART_IT_TC, ENABLE);
}

void send_float(u8 cmd, float* data)
{
    u8 *tmp = (u8 *) data;
	tx_buffer[tx_buffer_token]=cmd;
	update_token();

	tx_buffer[tx_buffer_token]=tmp[0];
	update_token();
	tx_buffer[tx_buffer_token]=tmp[1];
	update_token();
	tx_buffer[tx_buffer_token]=tmp[2];
	update_token();
	tx_buffer[tx_buffer_token]=tmp[3];
	update_token();

	tx_buffer[tx_buffer_token]=stop_byte1;
	update_token();
	tx_buffer[tx_buffer_token]=stop_byte2;
	update_token();

    tx_data_size=7;
	USART_ITConfig(USART3, USART_IT_TC, ENABLE);
}

void send_3_floats(u8 cmd, float* data1, float* data2, float* data3)
{
    u8 *tmp1 = (u8 *) data1;
	u8 *tmp2 = (u8 *) data2;
	u8 *tmp3 = (u8 *) data3;

	tx_buffer[tx_buffer_token]=cmd;
	update_token();

	tx_buffer[tx_buffer_token]=tmp1[0];
	update_token();
	tx_buffer[tx_buffer_token]=tmp1[1];
	update_token();
	tx_buffer[tx_buffer_token]=tmp1[2];
	update_token();
	tx_buffer[tx_buffer_token]=tmp1[3];
	update_token();

	tx_buffer[tx_buffer_token]=tmp2[0];
	update_token();
	tx_buffer[tx_buffer_token]=tmp2[1];
	update_token();
	tx_buffer[tx_buffer_token]=tmp2[2];
	update_token();
	tx_buffer[tx_buffer_token]=tmp2[3];
	update_token();

	tx_buffer[tx_buffer_token]=tmp3[0];
	update_token();
	tx_buffer[tx_buffer_token]=tmp3[1];
	update_token();
	tx_buffer[tx_buffer_token]=tmp3[2];
	update_token();
	tx_buffer[tx_buffer_token]=tmp3[3];
	update_token();

	tx_buffer[tx_buffer_token]=stop_byte1;
	update_token();
	tx_buffer[tx_buffer_token]=stop_byte2;
	update_token();

	tx_data_size=15;
	USART_ITConfig(USART3, USART_IT_TC, ENABLE);
}

extern vu16 ADC_ConvertedValue[];
extern float count;
extern float clock;
u32 clock2;
extern float d_cal;
extern float d_carrier;
extern float ADC_offset;
extern float current_input;
extern float PID;
extern float count_total;

void debugger_msg(void)
{					 
//	clock2++;
//	send_debug_msg(1, ADC_ConvertedValue[0]-ADC_offset);
	send_debug_msg(1, d_count);
//	send_debug_msg(1, current_input);
//	send_debug_msg(1, d_cal);
//	send_debug_msg(3, r_count);		  

	/*
	if (clock2%100<6)
	{
		send_debug_msg(clock2%100+1, clock2);
	}
	*/

}
