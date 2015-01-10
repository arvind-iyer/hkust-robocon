#include "system.h"

u8 enable_flag = 0;

/*** Encoders ***/
void encoder_init(void)
{
	hw_encoder_Init();
} 

void set_encoder_zero_all(void)
{
	hw_encoder_Count[0] = 0;
	hw_encoder_Count[1] = 0;
}

s32 read_encoder(char number)		//lm629 accumulates the total distance encoders have traveled
{
	return hw_encoder_Count[number];	
}

void set_encoder_zero(char number)
{
	hw_encoder_Count[number] = 0;
}

void read_encoders(int32_t* data_buf)
{
	switch(system_select)					//**Notice that there's no BREAK in this switch statement 
	{
		case T_SHAPE_SYSTEM:				//**need to read encoder1 and encoder0						
			data_buf[1] = read_encoder(1);
			if (data_buf[1] > 1000000000 || data_buf[1] < -1000000000)
			{
				set_encoder_zero(1);
				data_buf[1] = 0;
			}
		case ONE_ENCODER_SYSTEM:			//need to read encoder0 only
			data_buf[0] = read_encoder(0);
			if (data_buf[0] > 1000000000 || data_buf[0] < -1000000000)
			{
				set_encoder_zero(0);
				data_buf[0] = 0;
			}
	}
}
