#include "Temperature_ADC.h"

void  Adc_Init(void)
{    GPIO_InitTypeDef 	GPIO_InitStructure;

	//�ȳ�ʼ��IO��
 	RCC->APB2ENR|=1<<8;    	//ʹ��ADC1ʱ�� 
	RCC->AHB1ENR|=1<<0;    	//ʹ��PORTAʱ��	  
 
	
	
	   
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC->APB2RSTR|=1<<8;   	//ADCs��λ
	RCC->APB2RSTR&=~(1<<8);	//��λ����	 
	ADC->CCR=1<<16;			//ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz
 	ADC->CCR|=1<<23;		//ʹ���ڲ��¶ȴ�����
	
	ADC1->CR1=0;   			//CR1��������
	ADC1->CR2=0;   			//CR2��������
	ADC1->CR1|=0<<24;      	//12λģʽ
	ADC1->CR1|=0<<8;    	//��ɨ��ģʽ	
	
	ADC1->CR2&=~(1<<1);    	//����ת��ģʽ
 	ADC1->CR2&=~(1<<11);   	//�Ҷ���	
	ADC1->CR2|=0<<28;    	//�������
	
	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1|=0<<20;     	//1��ת���ڹ��������� Ҳ����ֻת����������1 			   
	//����ͨ��5�Ĳ���ʱ��
	ADC1->SMPR2&=~(7<<(3*5));		//ͨ��5����ʱ�����	  
 	ADC1->SMPR2|=7<<(3*5); 			//ͨ��5  480������,��߲���ʱ�������߾�ȷ��	

	ADC1->SMPR1&=~(7<<(3*(16-10))); //���ͨ��16ԭ��������	 
	ADC1->SMPR1|=7<<(3*(16-10));   	//ͨ��16 480����,��߲���ʱ�������߾�ȷ��
	
 	ADC1->CR2|=1<<0;	   	//����ADת����	  
}					  
//���ADCֵ
//ch:ͨ��ֵ 0~16
//����ֵ:ת�����
u16 Get_Adc(u8 ch)   
{
	//����ת������	  		 
	ADC1->SQR3&=0XFFFFFFE0;//��������1 ͨ��ch
	ADC1->SQR3|=ch;		  			    
	ADC1->CR2|=1<<30;       //��������ת��ͨ�� 
	while(!(ADC1->SR&1<<1));//�ȴ�ת������	 	   
	return ADC1->DR;		//����adcֵ	
}
//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_nms(5);
	}
	return temp_val/times;
}  
//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��,��λ:��.)
short Get_Temprate(void)
{
	u32 adcx;
	short result;
 	double temperate;
	adcx=Get_Adc_Average(ADC_CH_TEMP,1);	//��ȡͨ��16,20��ȡƽ��
	temperate=(float)adcx*(3.3/4096);		//��ѹֵ
	temperate=(temperate-0.76)/0.0025+25; 	//ת��Ϊ�¶�ֵ 
	result=temperate*=100;					//����100��.
	return result;
}