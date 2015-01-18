#include "battery.h"

#define DVDAx100 ((SAMPLE_BATTERY_VOLTAGE_2-SAMPLE_BATTERY_VOLTAGE_1)*100/(SAMPLE_BATTERY_ADC_2-SAMPLE_BATTERY_ADC_1))
	
static u16 adc_value = 0;

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
void battery_adc_init(void)
{
	battery_adc_gpio_init();
	adc_value = 0;
	{
		ADC_InitTypeDef ADC_InitStructure;					 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

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
}

void battery_adc_update(void)
{
	adc_value = ADC1->DR;
}

/**
  * @brief  Get the adc value of the voltage
  * @param  None
  * @retval ADC value
  */
u16 get_battery_adc(void)
{
	return adc_value;
}

/**
  * @brief  Calculating the voltage of power supply
  * @param  None
  * @retval voltage scaled by 100
  */
u16 get_voltage(void)
{ 
	return (u16)(DVDAx100 * (adc_value - SAMPLE_BATTERY_ADC_1) / 100 + SAMPLE_BATTERY_VOLTAGE_1);
}

/**
	* @brief Check the battery level and return the result enumator
	* @param None.
	* @retval The enum of battery check result.
	*/
BATTERY_CHECK_RESULT battery_check(void)
{
	battery_adc_update();
	if (get_voltage() <= BATTERY_USB_LEVEL) {
		return BATTERY_USB;
	} else if (get_voltage() <= BATTERY_SUPER_LOW_LEVEL) {
		return BATTERY_SUPER_LOW;
	} else if (get_voltage() <= BATTERY_LOW_LEVEL) {
		return BATTERY_LOW;
	} else {
		return BATTERY_OKAY;
	}
}
