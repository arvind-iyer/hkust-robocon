#include "dht11.h" 

void DHT11_IO_OUT(){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = DHT11_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(DHT11_PORT, &GPIO_InitStructure);

}
    
//RESET the DHT11 by hw
void DHT11_Rst(void)	    
{                  
	DHT11_IO_OUT(); 	//SET TO OUTPUT mode 
    GPIO_WriteBit(DHT11_PORT,DHT11_PIN,0); 	//pull down DQ 
    delay_nms(20);    	//��������18ms 
    GPIO_WriteBit(DHT11_PORT,DHT11_PIN,1); 	//DQ=1  
	delay_nus(30);     	//��������20~40us 
} 

void DHT11_IO_IN(){
	
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = DHT11_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DHT11_PORT, &GPIO_InitStructure);

}

//�ȴ�DHT11�Ļ�Ӧ 
//����1:δ��⵽DHT11�Ĵ��� 
//����0:���� 
u8 DHT11_Check(void) 	    
{    
u8 retry=0; 
	DHT11_IO_IN();//SET INPUT	  
	while (GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN) && retry < 85)//DHT11������40~80us 
	{ 
	retry++; 
	delay_nus(1); 
	};	  
	
if(retry>=100)return 1; 
else retry=0; 

	while (!GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN)&&retry<85)//DHT11���ͺ���ٴ�����40~80us 
	{ 
	retry++; 
	delay_nus(1); 
	}; 
if(retry>=100)return 1;	     
return 0; 
} 

//��DHT11��ȡһ��λ 
//����ֵ��1/0 
u8 DHT11_Read_Bit(void) 	  
{ 
 u8 retry=0; 
while(GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN)&&retry<100)//�ȴ���Ϊ�͵�ƽ 
{ 
retry++; 
delay_nus(1); 
} 
retry=0; 
while(!GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN)&&retry<100)//�ȴ���ߵ�ƽ 
{ 
retry++; 
delay_nus(1); 
} 
delay_nus(30);//wait 30us to see whether it is a long signal(1) or short signal(0)

if(GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN))return 1; //data coming from here

else return 0;	    
} 
//��DHT11��ȡһ���ֽ� 
//����ֵ������������ 
u8 DHT11_Read_Byte(void)     
{         
    u8 i,dat; 
    dat=0; 
for (i=0;i<8;i++)  
{ 
   	dat<<=1;  
    dat|=DHT11_Read_Bit(); 
    }	     
    return dat; 
} 
//��DHT11��ȡһ������ 
//temp:�¶�ֵ(��Χ:0~50��) 
//humi:ʪ��ֵ(��Χ:20%~90%) 
//����ֵ��0,����;1,��ȡʧ�� 
u8 DHT11_Read_Data(u8 *temp,u8 *humi)     
{   
u8 buf[5]; 
u8 i; 
DHT11_Rst(); 
if(DHT11_Check()==0) 
{ 
for(i=0;i<5;i++)//��ȡ40λ���� but the buf[3] and buf [1] is nth inside 
{ 
buf[i]=DHT11_Read_Byte(); 
} 
if((buf[0]+buf[2])==buf[4]) 
{ 
*humi=buf[0]; 
*temp=buf[2]; 
} 
}else return 1; 
return 0;	     
} 
//��ʼ��DHT11��IO�� DQ ͬʱ���DHT11�Ĵ��� 
//����1:������ 
//����0:����    	  
u8 DHT11_Init(void) 
{ 
DHT11_IO_OUT();
DHT11_Rst(); 
return DHT11_Check(); 
}  