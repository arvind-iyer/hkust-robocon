/* Includes ------------------------------------------------------------------*/
#include "configuration.h"	  

u16 ticks_img = 0;

int main(void)
{
	/* System clocks configuration ---------------------------------------------*/
	RCC_Configuration();
			  
	/* NVIC configuration ------------------------------------------------------*/
//	NVIC_Configuration();
	
	/* GPIO configuration ------------------------------------------------------*/
//	GPIO_Configuration();
	
	/* Configure the USART1 */
	//USART1_Configuration();
	CAN_Configuration();

	ticks_init();
	
	_delay_ms(100);

	/* Configure DMA */
	//DMA_Configuration();
	
	/* Configure ADC */
	//ADC_Configuration();
	
	/* Configure SPI1 as master*/
	//SPI_Configuration();
		
	//read_calibration_data();

	while(1)
	{
		if (ticks_img != get_ticks()) 
		{
			ticks_img = get_ticks();
		
			if(ticks_img % 5 == 0)
			{
				/*regular update*/
				//update_all();

				//printf("%ld\n\r",get_seconds());
				
				/*check if the user is requesting calibration*/
				if(calibration_flag_high || calibration_flag_low)	/*check whether it is already in calibration state*/
					prev_calibration_flag = 1;
				else
				 	prev_calibration_flag = 0;
		
				//calibration_flag_high = 1-GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12); 
				//calibration_flag_low = 1-GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11); 
		
				/*calibrating high voltage, keep the sensor bar above the white line*/
				if((calibration_flag_high) && (!prev_calibration_flag))	/*high voltage button down*/
				{
					/*print for debug*/
					read_calibration_data();
					/*
					printf("max voltage calibration : ");
					printf("previous data: ");
					for(i = 0; i < 32; i++)
					{
						printf("%ld ",stored_max_voltage[i]);
					}
					*/
					max_voltage_calibration();
					store_calibration_data();
					read_calibration_data();
					/*
					printf("new data");
					for(i = 0; i < 32; i++)
					{
					 	printf("%ld ",stored_max_voltage[i]);
					}
					printf(";\r\n");
					*/
				}
				/*calibrating low voltage, keep the sensor bar above the blue or red gamefield*/
				else if((calibration_flag_low)&&(!prev_calibration_flag))	/*low voltage button down*/
				{
					/*print for debug*/
					read_calibration_data();
					/*
					printf("min voltage calibration : ");
					printf("previous data: ");
					for(i = 0; i < 32; i++)
					{
						printf("%ld ",stored_min_voltage[i]);
					}
					*/
					min_voltage_calibration();
					store_calibration_data();
					read_calibration_data();
					/*
					printf("new data");
					for(i = 0; i < 32; i++)
					{
					 	printf("%ld ",stored_min_voltage[i]);
					}
					printf(";\r\n");
					*/
				}
				/*normal program*/
				else 
				{
					//GUI_debug();
					//PUTTY_debug();
				}
			}
			if(ticks_img % 20 == 4)
				Device2_TX();
		}
	}
}

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
