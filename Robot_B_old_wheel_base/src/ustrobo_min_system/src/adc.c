#include "adc.h"

#define ADC1_CHANNEL_COUNT      3

static u32 ADC1_Value[ADC1_CHANNEL_COUNT];

static u32* ADC_Channel_Value[ADC_CHANNEL_COUNT] = {
  0,                /* Channel 0 */
  0,                /* Channel 1 */
  0,                /* Channel 2 */
  0,                /* Channel 3 */
  0,                /* Channel 4 */
  0,                /* Channel 5 */
  0,                /* Channel 6 */
  0,                /* Channel 7 */
  0,                /* Channel 8 */
  0,                /* Channel 9 */
  &ADC1_Value[0],   /* Channel 10 */
  0,                /* Channel 11 */
  0,                /* Channel 12 */
  0,                /* Channel 13 */
  &ADC1_Value[2],   /* Channel 14 */
  0,                /* Channel 15 */
  &ADC1_Value[1],   /* Channel 16 */
  0                 /* Channel 17 */
};

void adc_init(void)
{
    /*** RCC Init ***/
    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
    /* Enable peripheral clocks ------------------------------------------------*/
    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* Enable ADC1, ADC2 and GPIOC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);


    
    /* DMA1 channel1 configuration ----------------------------------------------*/
    DMA_InitTypeDef DMA_InitStructure;  
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) &ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32) &ADC1_Value[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = ADC1_CHANNEL_COUNT;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    /* Enable DMA1 Channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
    /* ADC1 configuration ------------------------------------------------------*/
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = ADC1_CHANNEL_COUNT;
    ADC_Init(ADC1, &ADC_InitStructure);
    /* ADC1 regular channels configuration */ 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);    
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 2, ADC_SampleTime_239Cycles5);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 3, ADC_SampleTime_239Cycles5);
    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    /* Enable Vrefint channel17 */
    ADC_TempSensorVrefintCmd(ENABLE);

    /* Enable ADC1 reset calibration register */   
    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));
    
    /* Start ADC1 Software Conversion */ 
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* Test on DMA1 channel1 transfer complete flag */
    while(!DMA_GetFlagStatus(DMA1_FLAG_TC1));
    /* Clear DMA1 channel1 transfer complete flag */
    DMA_ClearFlag(DMA1_FLAG_TC1);
  
}

u32 get_adc_value(u8 channel)
{
  if (ADC_Channel_Value[channel] == 0) {
    return ADC_CHANNEL_UNUSED_VALUE;
  } else {
    return *(ADC_Channel_Value[channel]);
  }
}