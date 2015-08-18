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
    delay_nms(20);    	//拉低至少18ms 
    GPIO_WriteBit(DHT11_PORT,DHT11_PIN,1); 	//DQ=1  
	delay_nus(30);     	//主机拉高20~40us 
} 

void DHT11_IO_IN(){
	
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = DHT11_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DHT11_PORT, &GPIO_InitStructure);

}

//等待DHT11的回应 
//返回1:未检测到DHT11的存在 
//返回0:存在 
u8 DHT11_Check(void) 	    
{    
u8 retry=0; 
	DHT11_IO_IN();//SET INPUT	  
	while (GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN) && retry < 85)//DHT11会拉低40~80us 
	{ 
	retry++; 
	delay_nus(1); 
	};	  
	
if(retry>=100)return 1; 
else retry=0; 

	while (!GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN)&&retry<85)//DHT11拉低后会再次拉高40~80us 
	{ 
	retry++; 
	delay_nus(1); 
	}; 
if(retry>=100)return 1;	     
return 0; 
} 

//从DHT11读取一个位 
//返回值：1/0 
u8 DHT11_Read_Bit(void) 	  
{ 
 u8 retry=0; 
while(GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN)&&retry<100)//等待变为低电平 
{ 
retry++; 
delay_nus(1); 
} 
retry=0; 
while(!GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN)&&retry<100)//等待变高电平 
{ 
retry++; 
delay_nus(1); 
} 
delay_nus(30);//wait 30us to see whether it is a long signal(1) or short signal(0)

if(GPIO_ReadInputDataBit(DHT11_PORT,DHT11_PIN))return 1; //data coming from here

else return 0;	    
} 
//从DHT11读取一个字节 
//返回值：读到的数据 
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
//从DHT11读取一次数据 
//temp:温度值(范围:0~50°) 
//humi:湿度值(范围:20%~90%) 
//返回值：0,正常;1,读取失败 
u8 DHT11_Read_Data(u8 *temp,u8 *humi)     
{   
u8 buf[5]; 
u8 i; 
DHT11_Rst(); 
if(DHT11_Check()==0) 
{ 
for(i=0;i<5;i++)//读取40位数据 but the buf[3] and buf [1] is nth inside 
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
//初始化DHT11的IO口 DQ 同时检测DHT11的存在 
//返回1:不存在 
//返回0:存在    	  
u8 DHT11_Init(void) 
{ 
DHT11_IO_OUT();
DHT11_Rst(); 
return DHT11_Check(); 
}  