/* INCLUDES ---------------------------*/
#include "calibration.h"

u16 test_data[5]={0x0001,0x0002,0x0003,0x0004,0x0005};

/*CALIBRATION DATA DEFINITION*/
//extern vu16 ADC_ConvertedValue[160];
//extern vu16 ADC_ConvertedValue2[16];
u16 max_voltage[MAX_VOL_DATA_LENGTH];
u16 min_voltage[MIN_VOL_DATA_LENGTH]; 
u16 stored_max_voltage[MAX_VOL_DATA_LENGTH];
u16 stored_min_voltage[MIN_VOL_DATA_LENGTH];


uint16_t read_data_size()
{
	return *(u8*)(0x8002000);
}

void data_size_initialize()
{
	FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(0x8002000);
	FLASH_ProgramHalfWord((0x8002000),1); 
	FLASH_Lock();
}


void flash_write_halfword(uint16_t data)
{
	FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(0x8002000);
	FLASH_ProgramHalfWord((0x8002000),data); 
	FLASH_Lock();
}


void flash_write_array(uint16_t* data, uint16_t data_length)
{
	uint16_t count = 0;
	FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(0x8002000);
	while(count < data_length)
	{
		FLASH_ProgramHalfWord((0x8002000 + count*2),data[count]); 
		count++;
	}
	//lock flash
	FLASH_Lock();
}

void test_write()
{
	//u16 tick;
	flash_write_array(test_data, 5);
	//for(tick = 0; tick < 50000; tick++);
	//flash_write_array(test_data, 5);
}

void print_test()
{
    uint16_t count = 0;
	printf("\r\n The Five Data Is : \r\n");	
	while(count < 5)
    {
        printf("\r %d \r",*(u16*)(0x8002000 + count*2));      
        count++;
    }	
}

/*save the maximum voltage data IN MEMORY---------------------------*/
void max_voltage_calibration()
{
	u8 index = 0;
	for (index = 0; index < 32; index++)
	{
	 	max_voltage[index] = (u16)(All_Data_Buffer[index]);
	}
}

/*save the maximum voltage data IN MEMORY---------------------------*/
void min_voltage_calibration()
{
	u8 index = 0;
	for (index = 0; index < 32; index++)
	{
	 	min_voltage[index] = (u16)(All_Data_Buffer[index]);
	}
}

void store_calibration_data()
{
 	u16 temp[CAL_DATA_LENGTH];
	u8 index = 0;
	for (index = 0; index < CAL_DATA_LENGTH; index++)
	{
	 	if(index < MAX_VOL_DATA_LENGTH)
			temp[index] = max_voltage[index];
		else
			temp[index] = min_voltage[index-MAX_VOL_DATA_LENGTH];
	} 
	flash_write_array(temp, CAL_DATA_LENGTH);
}

void read_calibration_data()
{
	u8 index = 0;
	for(index = 0; index < CAL_DATA_LENGTH; index++)
	{
		if(index <  MAX_VOL_DATA_LENGTH)
		{
			/*fetch the maximum voltage data from the flash*/
			stored_max_voltage[index] = *(u16*)(0x8002000 + index*2);
		}
		else
		{
			/*fetch the minimum voltage data from the flash*/
			stored_min_voltage[index - MAX_VOL_DATA_LENGTH] = *(u16*)(0x8002000 + index*2);
		}
	}	
}

