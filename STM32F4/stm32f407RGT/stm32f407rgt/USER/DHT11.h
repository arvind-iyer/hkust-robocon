#ifndef __DHT11_H 
#define __DHT11_H  
#include "stm32f4xx.h"
#include "ticks.h"


//IO�������� 

////IO��������	    
#define	DHT11_DQ_OUT   //���ݶ˿�	PA0  
#define	DHT11_DQ_IN    //���ݶ˿�	PA0  

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_Pin_0

u8 DHT11_Init(void);//��ʼ��DHT11 
u8 DHT11_Read_Data(u8 *temp,u8 *humi);//��ȡ��ʪ�� 
u8 DHT11_Read_Byte(void);//����һ���ֽ� 
u8 DHT11_Read_Bit(void);//����һ��λ 
u8 DHT11_Check(void);//����Ƿ����DHT11 
void DHT11_Rst(void);//��λDHT11     
#endif 