#include "gpio.h"

GPIO
	/*** GPIOA ***/
	PA0 (GPIOA, GPIO_Pin_0),
	PA1 (GPIOA, GPIO_Pin_1),
	PA2 (GPIOA, GPIO_Pin_2),
	PA3 (GPIOA, GPIO_Pin_3),
	PA4 (GPIOA, GPIO_Pin_4),
	PA5 (GPIOA, GPIO_Pin_5),
	PA6 (GPIOA, GPIO_Pin_6),
	PA7 (GPIOA, GPIO_Pin_7),
	PA8 (GPIOA, GPIO_Pin_8),
	PA9 (GPIOA, GPIO_Pin_9),
	PA10 (GPIOA, GPIO_Pin_10),
	PA11 (GPIOA, GPIO_Pin_11),
	PA12 (GPIOA, GPIO_Pin_12),
	PA13 (GPIOA, GPIO_Pin_13),
	PA14 (GPIOA, GPIO_Pin_14),
	PA15 (GPIOA, GPIO_Pin_15),

	/*** GPIOB ***/
	PB0 (GPIOB, GPIO_Pin_0),
	PB1 (GPIOB, GPIO_Pin_1),
	PB2 (GPIOB, GPIO_Pin_2),
	PB3 (GPIOB, GPIO_Pin_3),
	PB4 (GPIOB, GPIO_Pin_4),
	PB5 (GPIOB, GPIO_Pin_5),
	PB6 (GPIOB, GPIO_Pin_6),
	PB7 (GPIOB, GPIO_Pin_7),
	PB8 (GPIOB, GPIO_Pin_8),
	PB9 (GPIOB, GPIO_Pin_9),
	PB10 (GPIOB, GPIO_Pin_10),
	PB11 (GPIOB, GPIO_Pin_11),
	PB12 (GPIOB, GPIO_Pin_12),
	PB13 (GPIOB, GPIO_Pin_13),
	PB14 (GPIOB, GPIO_Pin_14),
	PB15 (GPIOB, GPIO_Pin_15),

	/*** GPIOC ***/
	PC0 (GPIOC, GPIO_Pin_0),
	PC1 (GPIOC, GPIO_Pin_1),
	PC2 (GPIOC, GPIO_Pin_2),
	PC3 (GPIOC, GPIO_Pin_3),
	PC4 (GPIOC, GPIO_Pin_4),
	PC5 (GPIOC, GPIO_Pin_5),
	PC6 (GPIOC, GPIO_Pin_6),
	PC7 (GPIOC, GPIO_Pin_7),
	PC8 (GPIOC, GPIO_Pin_8),
	PC9 (GPIOC, GPIO_Pin_9),
	PC10 (GPIOC, GPIO_Pin_10),
	PC11 (GPIOC, GPIO_Pin_11),
	PC12 (GPIOC, GPIO_Pin_12),
	PC13 (GPIOC, GPIO_Pin_13),
	PC14 (GPIOC, GPIO_Pin_14),
	PC15 (GPIOC, GPIO_Pin_15),

	/*** GPIOE ***/
	PE0 (GPIOE, GPIO_Pin_0),
	PE1 (GPIOE, GPIO_Pin_1),
	PE2 (GPIOE, GPIO_Pin_2),
	PE3 (GPIOE, GPIO_Pin_3),
	PE4 (GPIOE, GPIO_Pin_4),
	PE5 (GPIOE, GPIO_Pin_5),
	PE6 (GPIOE, GPIO_Pin_6),
	PE7 (GPIOE, GPIO_Pin_7),
	PE8 (GPIOE, GPIO_Pin_8),
	PE9 (GPIOE, GPIO_Pin_9),
	PE10 (GPIOE, GPIO_Pin_10),
	PE11 (GPIOE, GPIO_Pin_11),
	PE12 (GPIOE, GPIO_Pin_12),
	PE13 (GPIOE, GPIO_Pin_13),
	PE14 (GPIOE, GPIO_Pin_14),
	PE15 (GPIOE, GPIO_Pin_15),

	/*** GPIOD ***/
	PD0 (GPIOD, GPIO_Pin_0),
	PD1 (GPIOD, GPIO_Pin_1),
	PD2 (GPIOD, GPIO_Pin_2),
	PD3 (GPIOD, GPIO_Pin_3),
	PD4 (GPIOD, GPIO_Pin_4),
	PD5 (GPIOD, GPIO_Pin_5),
	PD6 (GPIOD, GPIO_Pin_6),
	PD7 (GPIOD, GPIO_Pin_7),
	PD8 (GPIOD, GPIO_Pin_8),
	PD9 (GPIOD, GPIO_Pin_9),
	PD10 (GPIOD, GPIO_Pin_10),
	PD11 (GPIOD, GPIO_Pin_11),
	PD12 (GPIOD, GPIO_Pin_12),
	PD13 (GPIOD, GPIO_Pin_13),
	PD14 (GPIOD, GPIO_Pin_14),
	PD15 (GPIOD, GPIO_Pin_15)
;



GPIO::GPIO(GPIO_TypeDef* _gpio, uint16_t pin) : gpio(_gpio), gpio_pin(pin), gpio_is_inited(false) {}

uint8_t GPIO::get_pin_source() const
{
	uint8_t Pin_x = 0;
	uint16_t temp = gpio_pin;
	while (temp >>= 1) {
		++Pin_x;
	};
	return Pin_x;

}

/**
* @brief GPIO Pin initailizer
* @param gpio: The gpio pointer
* @param speed: GPIO speed with type "GPIOSpeed_TypeDef"
* @param rcc_init: True if the GPIO port rcc also needs to be initialized
* @retval None
*/
void GPIO::gpio_init(GPIOSpeed_TypeDef speed, GPIOMode_TypeDef mode, GPIOOType_TypeDef otype, GPIOPuPd_TypeDef pull)
{
	switch ((u32) gpio) {
		case ((u32)GPIOA):
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		break;
		case ((u32)GPIOB):
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		break;
		case ((u32)GPIOC):
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		break;
		case ((u32)GPIOD):
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		break;
		case ((u32)GPIOE):
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		break;
		case ((u32)GPIOF):
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
		break;
		case ((u32)GPIOG):
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
		break;
		case ((u32)GPIOH):
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
		break;
		case ((u32)GPIOI):
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
		break;
		case ((u32)GPIOJ):
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOJ, ENABLE);
		break;
		case ((u32)GPIOK):
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOK, ENABLE);
		break;
	}

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = speed;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_OType = otype;
	GPIO_InitStructure.GPIO_PuPd = pull;
	GPIO_InitStructure.GPIO_Pin = gpio_pin;
	GPIO_Init(gpio, &GPIO_InitStructure);
	gpio_is_inited = true;
}

/**
* @brief Read GPIO input value
* @param GPIO pointer
* @retval The GPIO Pin input value
*/
u8 GPIO::gpio_read_input() const
{
	return GPIO_ReadInputDataBit(gpio, gpio_pin);
}

/**
* @brief Read GPIO output value
* @param GPIO pointer
* @retval The GPIO Pin output value
*/
u8 GPIO::gpio_read_output() const
{
	return GPIO_ReadOutputDataBit(gpio, gpio_pin);
}

/**
* @brief Write GPIO value
* @param GPIO pointer
* @retval None
*/
void GPIO::gpio_write(BitAction BitVal) const
{
	GPIO_WriteBit(gpio, gpio_pin, BitVal);
}

