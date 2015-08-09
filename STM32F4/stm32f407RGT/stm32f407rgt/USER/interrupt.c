#include "interrupt.h"

uint8_t get_port_source(const GPIO* gpio)
{
	if(gpio->gpio == GPIOA)
		return EXTI_PortSourceGPIOA;
	else if(gpio->gpio == GPIOB)
		return EXTI_PortSourceGPIOB;
	else if(gpio->gpio == GPIOC)
		return EXTI_PortSourceGPIOC;
	else if(gpio->gpio == GPIOD)
		return EXTI_PortSourceGPIOD;
	else if(gpio->gpio == GPIOE)
		return EXTI_PortSourceGPIOE;
	else if(gpio->gpio == GPIOF)
		return EXTI_PortSourceGPIOF;
	else if(gpio->gpio == GPIOG)
		return EXTI_PortSourceGPIOG;
	else if(gpio->gpio == GPIOH)
		return EXTI_PortSourceGPIOH;
	else if(gpio->gpio == GPIOI)
		return EXTI_PortSourceGPIOI;
	else if(gpio->gpio == GPIOJ)
		return EXTI_PortSourceGPIOJ;
	else
		return EXTI_PortSourceGPIOK;
	
}

uint8_t get_pin_source(const GPIO* gpio)
{
	if(gpio->gpio_pin == GPIO_Pin_0)
		return EXTI_PinSource0;
	else if(gpio->gpio_pin == GPIO_Pin_1)
		return EXTI_PinSource1;
	else if(gpio->gpio_pin == GPIO_Pin_2)
		return EXTI_PinSource2;
	else if(gpio->gpio_pin == GPIO_Pin_3)
		return EXTI_PinSource3;
	else if(gpio->gpio_pin == GPIO_Pin_4)
		return EXTI_PinSource4;
	else if(gpio->gpio_pin == GPIO_Pin_5)
		return EXTI_PinSource5;
	else if(gpio->gpio_pin == GPIO_Pin_6)
		return EXTI_PinSource6;
	else if(gpio->gpio_pin == GPIO_Pin_7)
		return EXTI_PinSource7;
	else if(gpio->gpio_pin == GPIO_Pin_8)
		return EXTI_PinSource8;
	else if(gpio->gpio_pin == GPIO_Pin_9)
		return EXTI_PinSource9;
	else if(gpio->gpio_pin == GPIO_Pin_10)
		return EXTI_PinSource10;
	else if(gpio->gpio_pin == GPIO_Pin_11)
		return EXTI_PinSource11;
	else if(gpio->gpio_pin == GPIO_Pin_12)
		return EXTI_PinSource12;
	else if(gpio->gpio_pin == GPIO_Pin_13)
		return EXTI_PinSource13;
	else if(gpio->gpio_pin == GPIO_Pin_14)
		return EXTI_PinSource14;
	else 
		return EXTI_PinSource15;
}

enum IRQn get_irq_channel(const GPIO* gpio)
{
	if(gpio->gpio_pin == GPIO_Pin_0)
		return EXTI0_IRQn;
	else if(gpio->gpio_pin == GPIO_Pin_1)
		return EXTI1_IRQn; 
	else if(gpio->gpio_pin == GPIO_Pin_2)
		return EXTI2_IRQn; 
	else if(gpio->gpio_pin == GPIO_Pin_3)
		return EXTI3_IRQn; 
	else if(gpio->gpio_pin == GPIO_Pin_4)
		return EXTI4_IRQn; 
	else if(gpio->gpio_pin == GPIO_Pin_5 || gpio->gpio_pin == GPIO_Pin_6 || gpio->gpio_pin == GPIO_Pin_7 || gpio->gpio_pin == GPIO_Pin_8 || gpio->gpio_pin == GPIO_Pin_9 )
		return EXTI9_5_IRQn;
	else
		return EXTI15_10_IRQn;
}

/**
	* @brief:  Initialises interrupt structures for external GPIO interrupt
	* @params: 
	*/

void init_gpio_interrupt(const GPIO* gpio, EXTITrigger_TypeDef trigger_type)
{
	/* Set variables used */
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	/* Enable clock for GPIO */
	gpio_rcc_init(gpio);
	/* Enable clock for SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	/* Set pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = gpio->gpio_pin;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(gpio->gpio, &GPIO_InitStruct);
	
	/* Tell system that you will use PD0 for EXTI_Line0 */
	SYSCFG_EXTILineConfig(get_port_source(gpio), get_pin_source(gpio));
	
	/* PD0 is connected to EXTI_Line0 */
	EXTI_InitStruct.EXTI_Line = (uint32_t)gpio->gpio_pin;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = trigger_type;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);

	/* Add IRQ vector to NVIC */
	/* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
	NVIC_InitStruct.NVIC_IRQChannel = get_irq_channel(gpio);
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);
}

/* Set interrupt handlers */
/* Handle PD0 interrupt */

//put what you want to do in the IRQ handler

void EXTI3_IRQHandler(void) {          					//for GPIO interrupt 
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
		/* Do your stuff when PE3 is changed */
		buzzer_play_song(START_UP, 125, 0);
		Print("565656");
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

void EXTI15_10_IRQHandler(void) {						//for GPIO interrupt
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line14) != RESET) {
		/* Do your stuff when PC14 is changed */
		buzzer_play_song(START_UP, 125, 0);
		Print("ygiygig");
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line14);
	}
}
