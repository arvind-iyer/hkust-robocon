#include "delay.h"	
#ifndef __ADC_H
#define __ADC_H	
#include "sys.h" 
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//ADC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									   
////////////////////////////////////////////////////////////////////////////////// 

   	    
#define ADC_CH_TEMP  	16 		 	//ͨ��16,�ڲ��¶ȴ�����ר��ͨ��	   	    
	   									   
void Adc_Init(void); 				//ADC��ʼ��
u16  Get_Adc(u8 ch); 				//���ĳ��ͨ��ֵ 
u16 Get_Adc_Average(u8 ch,u8 times);//�õ�ĳ��ͨ����������������ƽ��ֵ  
short Get_Temprate(void);			//��ȡ�ڲ��¶ȴ�����ֵ 
#endif 
