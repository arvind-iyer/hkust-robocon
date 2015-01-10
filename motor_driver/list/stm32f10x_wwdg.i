#line 1 "FWlib\\SRC\\stm32f10x_wwdg.c"


















  

 
#line 1 ".\\FWlib\\inc\\stm32f10x_wwdg.h"



















  

 



 
#line 1 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"























 



 



 
    






  


 
  


 

#line 59 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"


















 





#line 91 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"







            
#line 106 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"





 






 
#line 125 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 



 



 
#line 144 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"




 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMPER_IRQn                 = 2,       
  RTC_IRQn                    = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      

#line 203 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 224 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 252 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 278 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"


  ADC1_2_IRQn                 = 18,      
  USB_HP_CAN1_TX_IRQn         = 19,      
  USB_LP_CAN1_RX0_IRQn        = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_IRQn               = 24,      
  TIM1_UP_IRQn                = 25,      
  TIM1_TRG_COM_IRQn           = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTCAlarm_IRQn               = 41,      
  USBWakeUp_IRQn              = 42,      
  TIM8_BRK_IRQn               = 43,      
  TIM8_UP_IRQn                = 44,      
  TIM8_TRG_COM_IRQn           = 45,      
  TIM8_CC_IRQn                = 46,      
  ADC3_IRQn                   = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_IRQn                   = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Channel1_IRQn          = 56,      
  DMA2_Channel2_IRQn          = 57,      
  DMA2_Channel3_IRQn          = 58,      
  DMA2_Channel4_5_IRQn        = 59       


#line 364 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 409 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 455 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"
} IRQn_Type;



 

#line 1 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cm3.h"
 




















 























  







 




 






 

 











#line 93 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cm3.h"

#line 1 "E:\\UST\\Robocon2012\\KEIL\\ARM\\RV31\\INC\\stdint.h"
 
 





 









#line 25 "E:\\UST\\Robocon2012\\KEIL\\ARM\\RV31\\INC\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 196 "E:\\UST\\Robocon2012\\KEIL\\ARM\\RV31\\INC\\stdint.h"

     







     










     











#line 260 "E:\\UST\\Robocon2012\\KEIL\\ARM\\RV31\\INC\\stdint.h"



 


#line 95 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cm3.h"
#line 1 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cmInstr.h"
 




















 





 



 


 




 







 







 






 








 







 







 









 









 



static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}









 



static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}











 









 









 









 











 











 











 







 














 










 









 






#line 589 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cmInstr.h"

   

#line 96 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cm3.h"
#line 1 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cmFunc.h"
 




















 




 



 


 

 
 






 



static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}








 



static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}








 



static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}








 



static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}








 



static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}








 



static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}








 



static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}








 



static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}








 



static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}








 



static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}








 



static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}

 







 







 








 



static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}








 



static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}

 






 



static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}








 



static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & 1);
}





#line 348 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 660 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cmFunc.h"

 


#line 97 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cm3.h"









 
#line 114 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cm3.h"

 





 








 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                              
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[8];                  
       uint32_t RESERVED0[24];                                   
  volatile uint32_t ICER[8];                  
       uint32_t RSERVED1[24];                                    
  volatile uint32_t ISPR[8];                  
       uint32_t RESERVED2[24];                                   
  volatile uint32_t ICPR[8];                  
       uint32_t RESERVED3[24];                                   
  volatile uint32_t IABR[8];                  
       uint32_t RESERVED4[56];                                   
  volatile uint8_t  IP[240];                  
       uint32_t RESERVED5[644];                                  
  volatile  uint32_t STIR;                     
}  NVIC_Type;                                               

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
  volatile uint32_t VTOR;                     
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
  volatile uint8_t  SHP[12];                  
  volatile uint32_t SHCSR;                    
  volatile uint32_t CFSR;                     
  volatile uint32_t HFSR;                     
  volatile uint32_t DFSR;                     
  volatile uint32_t MMFAR;                    
  volatile uint32_t BFAR;                     
  volatile uint32_t AFSR;                     
  volatile const  uint32_t PFR[2];                   
  volatile const  uint32_t DFR;                      
  volatile const  uint32_t ADR;                      
  volatile const  uint32_t MMFR[4];                  
  volatile const  uint32_t ISAR[5];                  
} SCB_Type;                                                

 












 






























 






 





















 









 


















 
































                                     









 









 









 















 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 






 


 
typedef struct
{
  volatile  union  
  {
    volatile  uint8_t    u8;                   
    volatile  uint16_t   u16;                  
    volatile  uint32_t   u32;                  
  }  PORT [32];                           
       uint32_t RESERVED0[864];                                 
  volatile uint32_t TER;                      
       uint32_t RESERVED1[15];                                  
  volatile uint32_t TPR;                      
       uint32_t RESERVED2[15];                                  
  volatile uint32_t TCR;                      
       uint32_t RESERVED3[29];                                  
  volatile uint32_t IWR;                      
  volatile uint32_t IRR;                      
  volatile uint32_t IMCR;                     
       uint32_t RESERVED4[43];                                  
  volatile uint32_t LAR;                      
  volatile uint32_t LSR;                      
       uint32_t RESERVED5[6];                                   
  volatile const  uint32_t PID4;                     
  volatile const  uint32_t PID5;                     
  volatile const  uint32_t PID6;                     
  volatile const  uint32_t PID7;                     
  volatile const  uint32_t PID0;                     
  volatile const  uint32_t PID1;                     
  volatile const  uint32_t PID2;                     
  volatile const  uint32_t PID3;                     
  volatile const  uint32_t CID0;                     
  volatile const  uint32_t CID1;                     
  volatile const  uint32_t CID2;                     
  volatile const  uint32_t CID3;                     
} ITM_Type;                                                

 



 
























 



 



 



 









   






 


 
typedef struct
{
       uint32_t RESERVED0;
  volatile const  uint32_t ICTR;                     



       uint32_t RESERVED1;

} InterruptType_Type;

 



 









   


#line 732 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cm3.h"






 


 
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;

 




































 






 







































 




 
 
 
#line 848 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cm3.h"

#line 855 "E:\\UST\\Robocon2012\\KEIL\\ARM\\CMSIS\\Include\\core_cm3.h"






 





 






 



 



 










 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          
  
  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR;                                                    
  reg_value &= ~((0xFFFFUL << 16) | (7UL << 8));              
  reg_value  =  (reg_value                       |
                (0x5FA << 16) | 
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR =  reg_value;
}








 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR & (7UL << 8)) >> 8);    
}








 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}













 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
}















 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;
 
  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}















 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;
  
  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR  = ((0x5FA << 16)      | 
                 (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR & (7UL << 8)) | 
                 (1UL << 2));                    
  __dsb(0xF);                                                                    
  while(1);                                                     
}

 



 



 











 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > (0xFFFFFFUL << 0))  return (1);             
                                                               
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL))->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL))->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL))->CTRL  = (1UL << 2) | 
                   (1UL << 1)   | 
                   (1UL << 0);                     
  return (0);                                                   
}



 



 



 

extern volatile int32_t ITM_RxBuffer;                     











 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((CoreDebug_Type *) (0xE000EDF0UL))->DEMCR & (1UL << 24))  &&       
      (((ITM_Type *) (0xE0000000UL))->TCR & (1UL << 0))                  &&       
      (((ITM_Type *) (0xE0000000UL))->TER & (1UL << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000UL))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000UL))->PORT[0].u8 = (uint8_t) ch;
  }  
  return (ch);
}










 
static __inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }
  
  return (ch); 
}









 
static __inline int32_t ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

 









 
#line 462 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"
#line 1 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\system_stm32f10x.h"


















 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 463 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"
#line 464 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



   

 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

 





 



    



 

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;



 

typedef struct
{
  uint32_t  RESERVED0;
  volatile uint16_t DR1;
  uint16_t  RESERVED1;
  volatile uint16_t DR2;
  uint16_t  RESERVED2;
  volatile uint16_t DR3;
  uint16_t  RESERVED3;
  volatile uint16_t DR4;
  uint16_t  RESERVED4;
  volatile uint16_t DR5;
  uint16_t  RESERVED5;
  volatile uint16_t DR6;
  uint16_t  RESERVED6;
  volatile uint16_t DR7;
  uint16_t  RESERVED7;
  volatile uint16_t DR8;
  uint16_t  RESERVED8;
  volatile uint16_t DR9;
  uint16_t  RESERVED9;
  volatile uint16_t DR10;
  uint16_t  RESERVED10; 
  volatile uint16_t RTCCR;
  uint16_t  RESERVED11;
  volatile uint16_t CR;
  uint16_t  RESERVED12;
  volatile uint16_t CSR;
  uint16_t  RESERVED13[5];
  volatile uint16_t DR11;
  uint16_t  RESERVED14;
  volatile uint16_t DR12;
  uint16_t  RESERVED15;
  volatile uint16_t DR13;
  uint16_t  RESERVED16;
  volatile uint16_t DR14;
  uint16_t  RESERVED17;
  volatile uint16_t DR15;
  uint16_t  RESERVED18;
  volatile uint16_t DR16;
  uint16_t  RESERVED19;
  volatile uint16_t DR17;
  uint16_t  RESERVED20;
  volatile uint16_t DR18;
  uint16_t  RESERVED21;
  volatile uint16_t DR19;
  uint16_t  RESERVED22;
  volatile uint16_t DR20;
  uint16_t  RESERVED23;
  volatile uint16_t DR21;
  uint16_t  RESERVED24;
  volatile uint16_t DR22;
  uint16_t  RESERVED25;
  volatile uint16_t DR23;
  uint16_t  RESERVED26;
  volatile uint16_t DR24;
  uint16_t  RESERVED27;
  volatile uint16_t DR25;
  uint16_t  RESERVED28;
  volatile uint16_t DR26;
  uint16_t  RESERVED29;
  volatile uint16_t DR27;
  uint16_t  RESERVED30;
  volatile uint16_t DR28;
  uint16_t  RESERVED31;
  volatile uint16_t DR29;
  uint16_t  RESERVED32;
  volatile uint16_t DR30;
  uint16_t  RESERVED33; 
  volatile uint16_t DR31;
  uint16_t  RESERVED34;
  volatile uint16_t DR32;
  uint16_t  RESERVED35;
  volatile uint16_t DR33;
  uint16_t  RESERVED36;
  volatile uint16_t DR34;
  uint16_t  RESERVED37;
  volatile uint16_t DR35;
  uint16_t  RESERVED38;
  volatile uint16_t DR36;
  uint16_t  RESERVED39;
  volatile uint16_t DR37;
  uint16_t  RESERVED40;
  volatile uint16_t DR38;
  uint16_t  RESERVED41;
  volatile uint16_t DR39;
  uint16_t  RESERVED42;
  volatile uint16_t DR40;
  uint16_t  RESERVED43;
  volatile uint16_t DR41;
  uint16_t  RESERVED44;
  volatile uint16_t DR42;
  uint16_t  RESERVED45;    
} BKP_TypeDef;
  


 

typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];

  CAN_FilterRegister_TypeDef sFilterRegister[14];



} CAN_TypeDef;



 
typedef struct
{
  volatile uint32_t CFGR;
  volatile uint32_t OAR;
  volatile uint32_t PRES;
  volatile uint32_t ESR;
  volatile uint32_t CSR;
  volatile uint32_t TXD;
  volatile uint32_t RXD;  
} CEC_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  volatile uint32_t CR;
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;



} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;	
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
       uint32_t RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
       uint32_t RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
       uint32_t RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
       uint32_t RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
       uint32_t RESERVED4[5];
  volatile uint32_t MMCTGFCR;
       uint32_t RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
       uint32_t RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
       uint32_t RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
       uint32_t RESERVED8[567];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
       uint32_t RESERVED9[9];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
#line 903 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;



 

typedef struct
{
  volatile uint32_t BTCR[8];   
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;
  volatile uint32_t SR2;
  volatile uint32_t PMEM2;
  volatile uint32_t PATT2;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR2; 
} FSMC_Bank2_TypeDef;  



 
  
typedef struct
{
  volatile uint32_t PCR3;
  volatile uint32_t SR3;
  volatile uint32_t PMEM3;
  volatile uint32_t PATT3;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR3; 
} FSMC_Bank3_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t PCR4;
  volatile uint32_t SR4;
  volatile uint32_t PMEM4;
  volatile uint32_t PATT4;
  volatile uint32_t PIO4; 
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;  
} AFIO_TypeDef;


 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t OAR1;
  uint16_t  RESERVED2;
  volatile uint16_t OAR2;
  uint16_t  RESERVED3;
  volatile uint16_t DR;
  uint16_t  RESERVED4;
  volatile uint16_t SR1;
  uint16_t  RESERVED5;
  volatile uint16_t SR2;
  uint16_t  RESERVED6;
  volatile uint16_t CCR;
  uint16_t  RESERVED7;
  volatile uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;










} RCC_TypeDef;



 

typedef struct
{
  volatile uint16_t CRH;
  uint16_t  RESERVED0;
  volatile uint16_t CRL;
  uint16_t  RESERVED1;
  volatile uint16_t PRLH;
  uint16_t  RESERVED2;
  volatile uint16_t PRLL;
  uint16_t  RESERVED3;
  volatile uint16_t DIVH;
  uint16_t  RESERVED4;
  volatile uint16_t DIVL;
  uint16_t  RESERVED5;
  volatile uint16_t CNTH;
  uint16_t  RESERVED6;
  volatile uint16_t CNTL;
  uint16_t  RESERVED7;
  volatile uint16_t ALRH;
  uint16_t  RESERVED8;
  volatile uint16_t ALRL;
  uint16_t  RESERVED9;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SR;
  uint16_t  RESERVED2;
  volatile uint16_t DR;
  uint16_t  RESERVED3;
  volatile uint16_t CRCPR;
  uint16_t  RESERVED4;
  volatile uint16_t RXCRCR;
  uint16_t  RESERVED5;
  volatile uint16_t TXCRCR;
  uint16_t  RESERVED6;
  volatile uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  volatile uint16_t I2SPR;
  uint16_t  RESERVED8;  
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SMCR;
  uint16_t  RESERVED2;
  volatile uint16_t DIER;
  uint16_t  RESERVED3;
  volatile uint16_t SR;
  uint16_t  RESERVED4;
  volatile uint16_t EGR;
  uint16_t  RESERVED5;
  volatile uint16_t CCMR1;
  uint16_t  RESERVED6;
  volatile uint16_t CCMR2;
  uint16_t  RESERVED7;
  volatile uint16_t CCER;
  uint16_t  RESERVED8;
  volatile uint16_t CNT;
  uint16_t  RESERVED9;
  volatile uint16_t PSC;
  uint16_t  RESERVED10;
  volatile uint16_t ARR;
  uint16_t  RESERVED11;
  volatile uint16_t RCR;
  uint16_t  RESERVED12;
  volatile uint16_t CCR1;
  uint16_t  RESERVED13;
  volatile uint16_t CCR2;
  uint16_t  RESERVED14;
  volatile uint16_t CCR3;
  uint16_t  RESERVED15;
  volatile uint16_t CCR4;
  uint16_t  RESERVED16;
  volatile uint16_t BDTR;
  uint16_t  RESERVED17;
  volatile uint16_t DCR;
  uint16_t  RESERVED18;
  volatile uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;
  uint16_t  RESERVED0;
  volatile uint16_t DR;
  uint16_t  RESERVED1;
  volatile uint16_t BRR;
  uint16_t  RESERVED2;
  volatile uint16_t CR1;
  uint16_t  RESERVED3;
  volatile uint16_t CR2;
  uint16_t  RESERVED4;
  volatile uint16_t CR3;
  uint16_t  RESERVED5;
  volatile uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;



 
  


 











 




#line 1295 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 1318 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



#line 1337 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"




















 
  


   

#line 1437 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
#line 1498 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
#line 1674 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 1681 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
 








 








 






#line 1717 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 











 











 













 






#line 1833 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"




#line 1853 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 





#line 1866 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 1885 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 1894 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 1902 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



















#line 1927 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"












 













#line 1959 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"





#line 1973 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 1980 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 1990 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"











 


















#line 2026 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2034 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



















#line 2059 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"












 













#line 2091 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"





#line 2105 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 2112 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 2122 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"











 








 








   
#line 2161 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 2256 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 2283 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"
 
 
 
 
 
 

 




































































 




































































 
#line 2445 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2463 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2481 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 2498 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2516 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2535 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 

 






 
#line 2562 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"






 








 









 








 








 









 










 




#line 2637 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 










#line 2668 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 2683 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2692 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

   
#line 2701 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2710 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 2725 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2734 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

   
#line 2743 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2752 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 2767 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2776 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

   
#line 2785 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2794 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 2809 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2818 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

   
#line 2827 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2836 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 2845 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 2854 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 2864 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 2928 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2963 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2998 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3033 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3068 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
#line 3135 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 



 









 
#line 3159 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"




 




 
#line 3175 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 3197 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
 





 
#line 3212 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"
 
#line 3219 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
#line 3268 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3290 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3312 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3334 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3356 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3378 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
 
 
 
 

 
#line 3414 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3444 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3454 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3478 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3502 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3526 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3550 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3574 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3598 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
#line 3699 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3708 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"















  
 
#line 3731 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
#line 3866 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3873 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3880 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3887 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"







 
#line 3901 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3908 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3915 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3922 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3929 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3936 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3944 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3951 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3958 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3965 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3972 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3979 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3987 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 3994 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 4001 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 4008 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 









































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 
 





 






 


 
#line 4150 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4160 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 


 


 
 
 
 
 

 
















 









#line 4208 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 

























 
#line 4251 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4265 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4275 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 




























 





















 




























 





















 
#line 4394 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
#line 4429 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"





#line 4440 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4448 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 4455 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 


 
 
 
 
 

 




 
#line 4477 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
#line 4539 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 
#line 4551 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"







 


 
 
 
 
 

 











#line 4589 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 











#line 4612 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 











#line 4635 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 











#line 4658 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 5055 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5064 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5073 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5084 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5094 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5104 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5114 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5125 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5135 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5145 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5155 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5166 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5176 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5186 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5196 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5207 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5217 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5227 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5237 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5248 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5258 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5268 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5278 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5289 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5299 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5309 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5319 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5330 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5340 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5350 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

#line 5360 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






#line 5408 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 5478 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5493 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5519 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
#line 5740 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5752 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 






 
#line 5769 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


#line 5913 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5925 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5937 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5949 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5961 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5973 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5985 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5997 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 

 


#line 6011 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6023 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6035 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6047 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6059 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6071 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6083 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6095 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6107 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6119 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6131 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6143 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6155 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6167 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6179 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6191 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 
 
 
 
 

 
 
#line 6211 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6222 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6240 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"











 





 





 
#line 6278 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 












 
#line 6299 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 6439 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6456 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6473 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6490 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6524 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6558 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6592 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6626 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6660 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6694 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6728 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6762 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6796 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6830 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6864 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6898 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6932 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6966 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7000 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7034 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7068 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7102 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7136 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7170 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7204 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7238 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7272 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7306 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7340 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7374 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7408 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7442 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
 
 
 
 

 









#line 7469 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7477 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7487 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
#line 7548 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7557 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"







 



#line 7578 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 



 


 
#line 7603 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7613 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 




 


 
 
 
 
 

 
#line 7639 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 


 



 
#line 7663 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7672 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"







 
#line 7692 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7703 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 
 
 
 
 

 


#line 7732 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 









#line 7766 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 
 
 
 
 

 









 


 


 





 
#line 7806 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 



#line 8270 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"



 

 

  

#line 1 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x_conf.h"













 

 



 
#line 1 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x_type.h"














 

 



 
 
typedef signed long  s32;
typedef signed short s16;
typedef signed char  s8;

typedef signed long  const sc32;   
typedef signed short const sc16;   
typedef signed char  const sc8;    

typedef volatile signed long  vs32;
typedef volatile signed short vs16;
typedef volatile signed char  vs8;

typedef volatile signed long  const vsc32;   
typedef volatile signed short const vsc16;   
typedef volatile signed char  const vsc8;    

typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef unsigned long  const uc32;   
typedef unsigned short const uc16;   
typedef unsigned char  const uc8;    

typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char  vu8;

typedef volatile unsigned long  const vuc32;   
typedef volatile unsigned short const vuc16;   
typedef volatile unsigned char  const vuc8;    

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

#line 73 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x_type.h"

 
 
 



 
#line 22 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x_conf.h"

 
 


 
 

 
 





 


 


 


 


 


 
#line 66 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x_conf.h"

 


 



 


 


 
#line 90 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x_conf.h"

 




 


 


 


 


 


 


 





 


 
#line 133 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x_conf.h"

 
#line 141 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x_conf.h"

 



 


 
#line 167 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x_conf.h"



 
#line 8281 "E:\\UST\\Robocon2012\\KEIL\\ARM\\INC\\ST\\STM32F10x\\stm32f10x.h"




 

















 









 

  

 

 
#line 28 ".\\FWlib\\inc\\stm32f10x_wwdg.h"



 



  



  
  


  



  
  


  
  
#line 63 ".\\FWlib\\inc\\stm32f10x_wwdg.h"



  



  



  


  



  
  
void WWDG_DeInit(void);
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler);
void WWDG_SetWindowValue(uint8_t WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(uint8_t Counter);
void WWDG_Enable(uint8_t Counter);
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);





  



  



  

 
#line 23 "FWlib\\SRC\\stm32f10x_wwdg.c"
#line 1 ".\\FWlib\\inc\\stm32f10x_rcc.h"



















  

 



 
#line 28 ".\\FWlib\\inc\\stm32f10x_rcc.h"



 



 



 

typedef struct
{
  uint32_t SYSCLK_Frequency;
  uint32_t HCLK_Frequency;
  uint32_t PCLK1_Frequency;
  uint32_t PCLK2_Frequency;
  uint32_t ADCCLK_Frequency;
}RCC_ClocksTypeDef;



 



 



 









  



 

#line 82 ".\\FWlib\\inc\\stm32f10x_rcc.h"


  



 

#line 113 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 127 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 149 ".\\FWlib\\inc\\stm32f10x_rcc.h"


  



 

#line 165 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 183 ".\\FWlib\\inc\\stm32f10x_rcc.h"




 



 







 



 

#line 211 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 








 



 

#line 238 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 254 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 278 ".\\FWlib\\inc\\stm32f10x_rcc.h"




  



 

#line 309 ".\\FWlib\\inc\\stm32f10x_rcc.h"




 



 

#line 327 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 352 ".\\FWlib\\inc\\stm32f10x_rcc.h"




 



 



 



 



 

void RCC_DeInit(void);
void RCC_HSEConfig(uint32_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLK1Config(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);
void RCC_USBCLKConfig(uint32_t RCC_USBCLKSource);
void RCC_ADCCLKConfig(uint32_t RCC_PCLK2);
void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCOConfig(uint8_t RCC_MCO);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);




 



 



  

 
#line 24 "FWlib\\SRC\\stm32f10x_wwdg.c"



 




 



 



 



 

 


 




 

 


 






 



 



 



 



 



 



 



 






 
void WWDG_DeInit(void)
{
  RCC_APB1PeriphResetCmd(((uint32_t)0x00000800), ENABLE);
  RCC_APB1PeriphResetCmd(((uint32_t)0x00000800), DISABLE);
}










 
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler)
{
  uint32_t tmpreg = 0;
   
  ((void)0);
   
  tmpreg = ((WWDG_TypeDef *) (((uint32_t)0x40000000) + 0x2C00))->CFR & ((uint32_t)0xFFFFFE7F);
   
  tmpreg |= WWDG_Prescaler;
   
  ((WWDG_TypeDef *) (((uint32_t)0x40000000) + 0x2C00))->CFR = tmpreg;
}







 
void WWDG_SetWindowValue(uint8_t WindowValue)
{
  uint32_t tmpreg = 0;
   
  ((void)0);
   
  tmpreg = ((WWDG_TypeDef *) (((uint32_t)0x40000000) + 0x2C00))->CFR & ((uint32_t)0xFFFFFF80);
   
  tmpreg |= WindowValue & ((uint8_t)0x7F);
   
  ((WWDG_TypeDef *) (((uint32_t)0x40000000) + 0x2C00))->CFR = tmpreg;
}





 
void WWDG_EnableIT(void)
{
  *(volatile uint32_t *) (((uint32_t)0x42000000) + ((((((uint32_t)0x40000000) + 0x2C00) - ((uint32_t)0x40000000)) + 0x04) * 32) + (0x09 * 4)) = (uint32_t)ENABLE;
}






 
void WWDG_SetCounter(uint8_t Counter)
{
   
  ((void)0);
  
 
  ((WWDG_TypeDef *) (((uint32_t)0x40000000) + 0x2C00))->CR = Counter & ((uint8_t)0x7F);
}






 
void WWDG_Enable(uint8_t Counter)
{
   
  ((void)0);
  ((WWDG_TypeDef *) (((uint32_t)0x40000000) + 0x2C00))->CR = ((uint32_t)0x00000080) | Counter;
}





 
FlagStatus WWDG_GetFlagStatus(void)
{
  return (FlagStatus)(((WWDG_TypeDef *) (((uint32_t)0x40000000) + 0x2C00))->SR);
}





 
void WWDG_ClearFlag(void)
{
  ((WWDG_TypeDef *) (((uint32_t)0x40000000) + 0x2C00))->SR = (uint32_t)RESET;
}



 



 



 

 
