#include "algorithm.h"
#include "delay.h"

u16 Filtered_Data_Buffer[16];	/*core 1 filtered data buffer*/
u16 All_Data_Buffer[32];
u16 result[32];
u16 result_threshold = 0;
u8 on_line = 0;
u8 vague_flag = 0;
u8 binary_result[32];	/*this array indicates the binary representation of the data after the threshold filter*/
u8 high_result_start = 0;
u8 high_result_end = 0;
u8 final_result = 0;

void average_filter()
{
	u8 index = 0;
	u8 channel_index = 0;
	u16 data_sum = 0;
	for(index = 0; index < 16; index++)
	{
		data_sum = 0;
		for(channel_index = 0; channel_index < 10; channel_index++)
		{
			data_sum += (u16)(ADC_ConvertedValue[channel_index*16+index]);
		}
		Filtered_Data_Buffer[index] = data_sum/16;
	}	
}


/*recombine the data received from core2*/
void data_recombination()
{
	u8 index = 0;
	for(index=0;index<16;index++)
	{
		All_Data_Buffer[index] = Filtered_Data_Buffer[index];
	}
	
	/*raw data(from core2) recombination*/
	for(index=0;index<16;index++)
	{
		ADC_ConvertedValue2[index] = ADC_ConvertedValue2_raw[index*2]*256+ADC_ConvertedValue2_raw[index*2+1];
	}
	
	for(index=16;index<32;index++)
	{
		All_Data_Buffer[index] = (u16)(ADC_ConvertedValue2[index-16]);
	}
}

/*debug in MATLAB GUI*/
void GUI_debug()
{	
	USART_TX_BYTE(USART1, START_BYTE);	
	//USART_OUT16(USART1, &All_Data_Buffer[0], 32);	
	USART_TX_BYTE(USART1, final_result);
	USART_OUT16(USART1, &result[0], 32);	
}

/*debug SPI communication*/
void PUTTY_debug()
{
	/*
	u8 i = 0;
	printf("SPI2 has received:");
	for(i = 16; i < 32; i++)
	{
		printf("%ld, ", All_Data_Buffer[i]);	
	}
	printf("\n\r");
	_delay_ms(50);
	*/

	u8 i = 0;
	for(i = 0; i < 32; i++)
	{
		printf("%ld ", result[i]);
	}
	printf("\n\r");
	_delay_ms(50);
}

/*linearization*/
void linearization()
{
	s32 temp[32];
	u8 index = 0;
	for(index = 0; index < 32; index++)
	{
		temp[index] =  (u32)(((u16)(All_Data_Buffer[index]) -stored_min_voltage[index]));
		temp[index] = temp[index]*10000 / (stored_max_voltage[index] - stored_min_voltage[index]);
		if(temp[index] < 0)
			result[index] = 0;
		else if(temp[index] > 10000)
			result[index] = 10000;
		else
			result[index] = (u16)(temp[index]);
	}
}

/*caculate the threshold to decide whether the result indicates high voltage or low voltage*/
void threshold_calculation()
{
	//assumption: expected threshold = 5000
	u8 index = 0;
	u16 min_result = 10000;	//no result can be larger than 10000
	u16 max_result = 0; //no result can be less than 0
	u16 min_high_result = 10000; //the minimum result > 5000
	u8 possible_high_result_count = 0;
  	
	//find the min_result & max_result
	for(index = 0; index < 32; index++)
	{
		//update min_result
		if(min_result > result[index])
		 	min_result = result[index];
		//update max_result
		if(max_result < result[index])
			max_result = result[index];
		//update min_high_result;
		if(min_high_result > result[index] && result[index] > 5000)
			min_high_result = result[index];
		if(result[index] > 5000)
			possible_high_result_count++;
	}
	if(possible_high_result_count == 0)	//no result > 5000
	{
		result_threshold = (max_result + min_result)/2; 
		on_line = 0;
		vague_flag = 1;	
		//PAY ATTENTION: since in this case, we cannot determine
		//whether the sensor bar is high above the gound or
		//there's no white line detected, we set this vague_flag
		//and will make a prediction.(NOT ACCURATE)
	}
	else if(possible_high_result_count < 16)	//less than 16 results > 5000
	{
	 	result_threshold = (min_high_result + min_result)/2;
		on_line = 0;	//for sure
		vague_flag = 0;
	}
	else 	//over 16 results > 5000 OBVIOUSLY
	{
	 	on_line = 1;
		vague_flag = 0;
	}
}

void threshold_filter()
{
	u8 index = 0;
	for(index = 0; index < 32; index++)
	{
		if(result[index] > result_threshold)
			binary_result[index] = 1;
		else
			binary_result[index] = 0;
	}
}

/*longest continuous subarray:
 calculate the starting and ending indices of the subarray */
void longest_continuous_subarray()
{
	u8 start_index = 0;
	u8 end_index = 0;
	u8 max_length = 0;
	u8 index = 0;
	for(index = 0; index < 32; index++)
	{
		if(binary_result[index])
			end_index = index;
		else
		{
			start_index = index + 1;
			end_index = start_index;
		}			

		if(max_length < end_index - start_index + 1)
		{
			max_length = end_index - start_index + 1;
			high_result_start = start_index;
			high_result_end = end_index;
		}	
	}		
}

void update_all()
{
	average_filter();
	data_recombination(); 
	linearization();
	threshold_calculation();
	threshold_filter();
	longest_continuous_subarray();
	final_result = (high_result_start + high_result_end)/2;
}




