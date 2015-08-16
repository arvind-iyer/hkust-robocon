/**
  ******************************************************************************
  * @file    ADC.c
  * @author  Cheung Ngai { JOHN }
  * @version V1.0.0
  * @date    17-AUG-2015
  * @brief   This file provide you a ADC with DMA. It helps you to read ADC signal from any ADC pin

  ******************************************************************************
  * @attention
  *
  * This source is designed for application use. Unless necessary, try NOT to
	* modify the function definition. The constants which are more likely to 
	* vary among different schematics have been placed as pre-defined constant
	* (i.e., "#define") in the header file.
	*
  ******************************************************************************
  */
	


#include "adc.h"
#define Total_adc_count  2



u16 ADC1_level[Total_adc_count+1];


void ADC1_init(void)
{/* ADC init sturcture*/
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;   	
	
/*Enable clock for adc,dma,gpio*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/*Define and Enable GPIO*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure); 

	/*DMA init*/
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1_level[1];//to make it easier to see, the array start from 1
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = Total_adc_count;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/*ADC init*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = Total_adc_count;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_DMACmd(ADC1, ENABLE);
	
	//put here for more adc channel that you need to read
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 2, ADC_SampleTime_480Cycles);//temperature sensor
	//put more if needed, rank number cant be the same
	
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
    ADC_SoftwareStartConv(ADC1); 
	ADC_TempSensorVrefintCmd(ENABLE);
}


u16 ADC_raw_data(int RANK){
	return ADC1_level[RANK];
}

float get_battery()    //return voltage
{
float voltage_factor=0.0026996337;				//factor = 3300 / 4095 * total resistance / gnd_resistance / 1000
	return ADC1_level[1]*voltage_factor;		//3.176039647=3300/4095*67/17
	
}

float get_temperature() //return degree celcius
{
	
	return (ADC1_level[2]*0.805860805-760)/2500+25;//0.805860805=3300/4095

	   //Temperature (in °„C) = {(VSENSE ®C V25) / Avg_Slope} + 25
}
