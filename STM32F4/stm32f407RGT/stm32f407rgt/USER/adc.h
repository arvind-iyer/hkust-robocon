
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"




/*define !!!!!!must not!!!!!!! change*/
#define Total_adc_count  16
#define ADC_channel_10_CLOCK   RCC_AHB1Periph_GPIOC
#define ADC_channel_10_pin     GPIO_Pin_0





/*define ??????can??????? change
 *1=ON, 0=OFF
*/


#define ADC_channel_1_enable  1
#define ADC_channel_2_enable  1
#define ADC_channel_3_enable  1
#define ADC_channel_4_enable  1
#define ADC_channel_5_enable  1
#define ADC_channel_6_enable  1
#define ADC_channel_7_enable  1
#define ADC_channel_8_enable  1
#define ADC_channel_9_enable  1
#define ADC_channel_10_enable  1
#define ADC_channel_11_enable  1
#define ADC_channel_12_enable  1
#define ADC_channel_13_enable  1
#define ADC_channel_14_enable  1
#define ADC_channel_15_enable  1
#define ADC_channel_16_enable  0








void ADC1_init(void);
float get_battery();
u16 raw_data();
float get_temperature();
u16 get_temp_level();