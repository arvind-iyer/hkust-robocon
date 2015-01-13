#include "battery.h"

#define VOLTAGE_RATIO (u16)(100* (float)(ADC_OUTPUT_2 - ADC_OUTPUT_1)/(BATTERY_VOLTAGE_2 - BATTERY_VOLTAGE_1)) 
//Following values are use to convert the Voltage vs ADC relationship 
//to a Voltage vs temperature in celsius linear relationship
#define V25   (u16)(ADC_OUTPUT_1 * (0.1*V25_TYP/BATTERY_VOLTAGE_1))
#define Avg_Slope  (u16)( ADC_OUTPUT_1 * (0.0001*AVG_SLOPE_TYP/BATTERY_VOLTAGE_1))
u16 adc_value;

/**
  * @brief  Inintialization of DMA for voltage calculation
  * @param  None
  * @retval None
  */
void battery_dma_init(void)
{
    DMA_InitTypeDef DMA_InitStructure;    
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
		adc_value=0;					 
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)0x4001244C;			// address of ADC1 data register	
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&adc_value; //result would be assigned to battery_sample_value
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

/**
  * @brief  Inintialization of ADC for voltage calculation
  * @param  None
  * @retval None
  */
void battery_adc_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;			   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @brief  Inintialization of voltage calculation
  * @param  None
  * @retval None
  */
void adc_init(void)
{
	ADC_InitTypeDef ADC_InitStructure;					 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	battery_adc_gpio_init();
	battery_dma_init();	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
	ADC_TempSensorVrefintCmd(ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);  
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
  * @brief  Calculating the voltage of power supply
  * @param  None
  * @retval voltage scaled by 100
  */
u16 get_voltage(void)
{ 
	//To calculate voltage based on supplied data
	
	//Using equation for voltage V = ((adc_value+ADC_OFFSET)/VOLTAGE_RATIO)
	u16 ADC_OFFSET = BATTERY_VOLTAGE_1 * VOLTAGE_RATIO / 100 - ADC_OUTPUT_1;
	return (u16)(100 * (adc_value + ADC_OFFSET) / VOLTAGE_RATIO);  
}
/**
  * @brief  Calculating the temperature of semiconductor surface using the ADC
  * @param  None
  * @retval Temperature in Celsius
  */
u16 get_temp(void)
{
	
	return  (uint16_t)((u16)((V25-adc_value)/Avg_Slope)/25);
}
