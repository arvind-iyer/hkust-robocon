#include "adc_app.h"

const s16 BATTERY_SLOPEx100 = ((SAMPLE_BATTERY_VOLTAGE_2-SAMPLE_BATTERY_VOLTAGE_1)*100/(SAMPLE_BATTERY_ADC_2-SAMPLE_BATTERY_ADC_1));
const s16 TEMP_SLOPEx100 = ((SAMPLE_TEMP_VAL_2-SAMPLE_TEMP_VAL_1)*100/(SAMPLE_TEMP_ACD_2-SAMPLE_TEMP_ADC_1));
void battery_init(void)
{
  gpio_init(BATTERY_GPIO, GPIO_Speed_2MHz, GPIO_Mode_AIN, 1);
}

s16 get_voltage(void)
{
  u32 val = get_adc_value(BATTERY_ADC_CHANNEL);
  if (val == ADC_CHANNEL_UNUSED_VALUE || val == 0) {
    return 0;
  }
  //return get_adc_value(BATTERY_ADC_CHANNEL);
  return (s16)(BATTERY_SLOPEx100*(s16)(val - SAMPLE_BATTERY_ADC_1) / 100 + SAMPLE_BATTERY_VOLTAGE_1);
}

/**
	* @brief Check the battery level and return the result enumator
	* @param None.
	* @retval The enum of battery check result.
	*/
BATTERY_CHECK_RESULT battery_check(void)
{
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

s16 get_temperature(void)
{
  u32 val = get_adc_value(ADC_Channel_TempSensor);
  if (val == ADC_CHANNEL_UNUSED_VALUE || val == 0) {
    return 0;
  }
  //return get_adc_value(BATTERY_ADC_CHANNEL);
  return (s16)(TEMP_SLOPEx100*(s16)(val - SAMPLE_TEMP_ADC_1) / 100 + SAMPLE_TEMP_VAL_1);
}

u32 get_sensor(void)
{
  u32 val = get_adc_value(LASER_ADC_CHANNEL); 
  if (val == ADC_CHANNEL_UNUSED_VALUE || val == 0) {
    return 0;
  }
  //return get_adc_value(BATTERY_ADC_CHANNEL);
  return (s16)val;
}