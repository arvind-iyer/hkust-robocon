#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H

/*----------------------INCLUDES----------------------------*/
#include "stm32f10x.h"	  
#include "stm32f10x_conf.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_spi.h"

#include "misc.h"
#include "stdio.h"

#include "calibration.h"
#include "delay.h"
#include "algorithm.h"
#include "ticks.h"
#include "Robocon_CAN.h"

/* Private definition -----------------------------------------------------*/
#define ADC1_DR_Address    ((u32)0x4001244C)
#define SPI2_DR_Base  ((u32)0x4000380C)

/* Variables shared -------------------------------------------------------*/
extern vu16 ADC_ConvertedValue[160];
extern vu16 ADC_ConvertedValue2[16];
extern u8 ADC_ConvertedValue2_raw[32];
extern u16 All_Data_Buffer[32];
extern u8 calibration_flag_high;
extern u8 calibration_flag_low;
extern u8 prev_calibration_flag;
extern u16 stored_max_voltage[32];
extern u16 stored_min_voltage[32];
extern u16 result[32];
extern u8 final_result;

/* Private function prototypes -----------------------------------------------*/
void DMA_Configuration(void);
void ADC_Configuration(void);
void SPI_Configuration(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART1_Configuration(void);
void USART_OUT16(USART_TypeDef* USARTx, uc16 *Data,uint16_t Len);
void USART_OUT(USART_TypeDef* USARTx, uc8 *Data,uint16_t Len);
void USART_TX_BYTE(USART_TypeDef* USARTx, uc8 Data);

/* MATLAB GUI debug parameters ----------------------------------------------*/
#define START_BYTE	0xAA

#endif
