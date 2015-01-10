#include "variables.h"
#include "main.h"
//#include "uart.h"

#define stop_byte1			0xfe
#define stop_byte2			0xff

#define send_uint8_cmd		0x20
#define send_int8_cmd		0x24
#define send_uint16_cmd		0x28
#define send_int16_cmd		0x2b


unsigned char 		uint8[4];
unsigned short int	uint16[4];
signed char			int8[4];
signed short int	int16[4];


void uart_send_byte	(unsigned char data)
{
	//UART0_SendByte(data);
	USART_SendData(USART3, data);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

void send_uint8	(char index)
{
	uart_send_byte (send_uint8_cmd + index);
	uart_send_byte (uint8[index - 1]);
	uart_send_byte (stop_byte1);
	uart_send_byte (stop_byte2);
}

void send_int8	(char index)
{
	uart_send_byte (send_int8_cmd + index);
	uart_send_byte (int8[index - 1]);
	uart_send_byte (stop_byte1);
	uart_send_byte (stop_byte2);
}

void send_uint16(char index)
{
	uart_send_byte (send_uint16_cmd + index);
	uart_send_byte (uint16[index - 1] >> 8);
	uart_send_byte (uint16[index - 1] & 0xff);
	uart_send_byte (stop_byte1);
	uart_send_byte (stop_byte2);
}

void send_int16	(char index)
{
	uart_send_byte (send_int16_cmd + index);
	uart_send_byte (int16[index - 1] >> 8);		   
	uart_send_byte (int16[index - 1] & 0xff);
	uart_send_byte (stop_byte1);
	uart_send_byte (stop_byte2);
}

void send_float32 (char index)
{
	unsigned char *tmp = conversion;
	tmp = (unsigned char *)&float32[index - 1];
	uart_send_byte (send_float32_cmd + index);
	uart_send_byte (tmp[0]);		   
	uart_send_byte (tmp[1]);
	uart_send_byte (tmp[2]);		   
	uart_send_byte (tmp[3]);
	uart_send_byte (stop_byte1);
	uart_send_byte (stop_byte2);
}

void set_uint8		(char index, unsigned char data)		{ uint8[index - 1] = data; }
void set_int8		(char index, signed char data)			{ int8[index - 1] = data; }
void set_uint16		(char index, unsigned short int data)	{ uint16[index - 1] = data; }
void set_int16		(char index, signed short int data)		{ int16[index - 1] = data; }
void set_float32	(char index, float data)				{ float32[index - 1] = data; }

void set_and_send_uint8 	(char index, unsigned char data) 		{ set_uint8 (index, data);		send_uint8 (index); }
void set_and_send_int8 		(char index, signed char data) 			{ set_int8 (index, data);		send_int8 (index); }
void set_and_send_uint16	(char index, unsigned short int data)	{ set_uint16 (index, data);		send_uint16 (index); }
void set_and_send_int16		(char index, signed short int data) 	{ set_int16 (index, data);		send_int16 (index); }
void set_and_send_float32	(char index, float data)				{ set_float32 (index, data);	send_float32 (index); }
