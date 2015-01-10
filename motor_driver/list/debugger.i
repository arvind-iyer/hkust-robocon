#line 1 "user\\debugger.c"
#line 1 "user\\debugger.h"



#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 985 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 5 "user\\debugger.h"
#line 1 "user\\stm32f10x.h"





















 



 



 
 


  


 
  


 















 

#line 67 "user\\stm32f10x.h"




 




 





 

 

 

 






 



 



 




 
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
#line 190 "user\\stm32f10x.h"
} IRQn_Type;



 

#line 1 ".\\FWlib\\inc\\core_cm3.h"
 




















 





































 

 
 
 
 
 
 
 
 








 











#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
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


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 91 ".\\FWlib\\inc\\core_cm3.h"

















 

#line 117 ".\\FWlib\\inc\\core_cm3.h"





 


 





 
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

 



 








   


#line 614 ".\\FWlib\\inc\\core_cm3.h"





 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;

 




































 






 






































   


 
#line 721 ".\\FWlib\\inc\\core_cm3.h"

#line 728 ".\\FWlib\\inc\\core_cm3.h"






   




 





#line 758 ".\\FWlib\\inc\\core_cm3.h"


 


 




#line 783 ".\\FWlib\\inc\\core_cm3.h"


 
 
 
 








 
extern uint32_t __get_PSP(void);








 
extern void __set_PSP(uint32_t topOfProcStack);








 
extern uint32_t __get_MSP(void);








 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 933 ".\\FWlib\\inc\\core_cm3.h"





 








 
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





#line 1445 ".\\FWlib\\inc\\core_cm3.h"







 
 

 











 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          
  
  reg_value  =  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR;                                                    
  reg_value &= ~((0xFFFFul << 16) | (7ul << 8));              
  reg_value  =  (reg_value                       |
                (0x5FA << 16) | 
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR =  reg_value;
}








 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) >> 8);    
}








 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000) + 0x0100))->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}















 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
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



 












 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > (0xFFFFFFul << 0))  return (1);             
                                                               
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->LOAD  = (ticks & (0xFFFFFFul << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);   
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->CTRL  = (1ul << 2) | 
                   (1ul << 1)   | 
                   (1ul << 0);                     
  return (0);                                                   
}






 





 
static __inline void NVIC_SystemReset(void)
{
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR  = ((0x5FA << 16)      | 
                 (((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) | 
                 (1ul << 2));                    
  __dsb(0);                                                                    
  while(1);                                                     
}

   



 






 
 

extern volatile int ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((CoreDebug_Type *) (0xE000EDF0))->DEMCR & (1ul << 24))  &&       
      (((ITM_Type *) (0xE0000000))->TCR & (1ul << 0))                  &&       
      (((ITM_Type *) (0xE0000000))->TER & (1ul << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000))->PORT[0].u8 = (uint8_t) ch;
  }  
  return (ch);
}










 
static __inline int ITM_ReceiveChar (void) {
  int ch = -1;                                

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }
  
  return (ch); 
}









 
static __inline int ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

   






   



 
#line 197 "user\\stm32f10x.h"
#line 1 "user\\system_stm32f10x.h"


















 



 





 



 




 

extern const uint32_t SystemFrequency;                    
extern const uint32_t SystemFrequency_SysClk;             
extern const uint32_t SystemFrequency_AHBClk;             
extern const uint32_t SystemFrequency_APB1Clk;            
extern const uint32_t SystemFrequency_APB2Clk;            



 



 



 



 



 



 
  
extern void SystemInit(void);


 



 
#line 198 "user\\stm32f10x.h"
#line 199 "user\\stm32f10x.h"



   

 
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

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

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



 
  


 









 




#line 911 "user\\stm32f10x.h"

#line 928 "user\\stm32f10x.h"



#line 947 "user\\stm32f10x.h"














 
  


   

#line 1029 "user\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
#line 1090 "user\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
#line 1266 "user\\stm32f10x.h"

 




 








 










 
#line 1302 "user\\stm32f10x.h"






 











 










 














 
#line 1362 "user\\stm32f10x.h"








 






 
#line 1395 "user\\stm32f10x.h"

 
#line 1412 "user\\stm32f10x.h"

 
#line 1434 "user\\stm32f10x.h"

 
#line 1443 "user\\stm32f10x.h"

 
#line 1460 "user\\stm32f10x.h"

 
#line 1482 "user\\stm32f10x.h"

 








 








   
#line 1511 "user\\stm32f10x.h"

 
 
 
 
 

 




































































 




































































 
#line 1673 "user\\stm32f10x.h"

 
#line 1691 "user\\stm32f10x.h"

 
#line 1709 "user\\stm32f10x.h"

#line 1726 "user\\stm32f10x.h"

 
#line 1744 "user\\stm32f10x.h"

 
#line 1763 "user\\stm32f10x.h"

 

 






 
#line 1790 "user\\stm32f10x.h"






 








 









 








 








 









 










 




#line 1865 "user\\stm32f10x.h"






 





 





 
#line 1891 "user\\stm32f10x.h"

 
#line 1900 "user\\stm32f10x.h"

   
#line 1909 "user\\stm32f10x.h"

 
#line 1918 "user\\stm32f10x.h"

 





 
#line 1933 "user\\stm32f10x.h"

 
#line 1942 "user\\stm32f10x.h"

   
#line 1951 "user\\stm32f10x.h"

 
#line 1960 "user\\stm32f10x.h"

 





 
#line 1975 "user\\stm32f10x.h"

 
#line 1984 "user\\stm32f10x.h"

   
#line 1993 "user\\stm32f10x.h"

 
#line 2002 "user\\stm32f10x.h"

 





 
#line 2017 "user\\stm32f10x.h"

 
#line 2026 "user\\stm32f10x.h"

   
#line 2035 "user\\stm32f10x.h"

 
#line 2044 "user\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 2108 "user\\stm32f10x.h"

 
#line 2143 "user\\stm32f10x.h"

 
#line 2178 "user\\stm32f10x.h"

 
#line 2213 "user\\stm32f10x.h"

 
#line 2248 "user\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
#line 2315 "user\\stm32f10x.h"

 



 









 
#line 2339 "user\\stm32f10x.h"




 




 
#line 2355 "user\\stm32f10x.h"

 





 
#line 2377 "user\\stm32f10x.h"

 
 





 
#line 2392 "user\\stm32f10x.h"
 
#line 2399 "user\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
#line 2447 "user\\stm32f10x.h"

 
#line 2468 "user\\stm32f10x.h"

 
#line 2489 "user\\stm32f10x.h"

 
#line 2510 "user\\stm32f10x.h"

 
#line 2531 "user\\stm32f10x.h"

 
#line 2552 "user\\stm32f10x.h"

 
 
 
 
 

 
#line 2588 "user\\stm32f10x.h"

 
#line 2618 "user\\stm32f10x.h"

 
#line 2628 "user\\stm32f10x.h"















 
#line 2652 "user\\stm32f10x.h"















 
#line 2676 "user\\stm32f10x.h"















 
#line 2700 "user\\stm32f10x.h"















 
#line 2724 "user\\stm32f10x.h"















 
#line 2748 "user\\stm32f10x.h"















 
#line 2772 "user\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
#line 2873 "user\\stm32f10x.h"

#line 2882 "user\\stm32f10x.h"















  
 
#line 2905 "user\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
#line 3040 "user\\stm32f10x.h"

#line 3047 "user\\stm32f10x.h"

#line 3054 "user\\stm32f10x.h"

#line 3061 "user\\stm32f10x.h"







 
#line 3075 "user\\stm32f10x.h"

#line 3082 "user\\stm32f10x.h"

#line 3089 "user\\stm32f10x.h"

#line 3096 "user\\stm32f10x.h"

#line 3103 "user\\stm32f10x.h"

#line 3110 "user\\stm32f10x.h"

 
#line 3118 "user\\stm32f10x.h"

#line 3125 "user\\stm32f10x.h"

#line 3132 "user\\stm32f10x.h"

#line 3139 "user\\stm32f10x.h"

#line 3146 "user\\stm32f10x.h"

#line 3153 "user\\stm32f10x.h"

 
#line 3161 "user\\stm32f10x.h"

#line 3168 "user\\stm32f10x.h"

#line 3175 "user\\stm32f10x.h"

#line 3182 "user\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 









































 



 


 


 


 


 


 


 



 



 



 


 


 
 
 
 
 

 
















 









#line 3332 "user\\stm32f10x.h"

 

























 
#line 3375 "user\\stm32f10x.h"

 
#line 3389 "user\\stm32f10x.h"

 
#line 3399 "user\\stm32f10x.h"

 




























 





















 




























 





















 
#line 3517 "user\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
#line 3552 "user\\stm32f10x.h"





#line 3563 "user\\stm32f10x.h"

 
#line 3571 "user\\stm32f10x.h"

#line 3578 "user\\stm32f10x.h"

 


 
 
 
 
 

 




 
#line 3600 "user\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
#line 3662 "user\\stm32f10x.h"



 
#line 3674 "user\\stm32f10x.h"







 


 
 
 
 
 

 











#line 3711 "user\\stm32f10x.h"

 











#line 3733 "user\\stm32f10x.h"

 











#line 3755 "user\\stm32f10x.h"

 











#line 3777 "user\\stm32f10x.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 4174 "user\\stm32f10x.h"

 
#line 4183 "user\\stm32f10x.h"

 
#line 4192 "user\\stm32f10x.h"

 
#line 4203 "user\\stm32f10x.h"

#line 4213 "user\\stm32f10x.h"

#line 4223 "user\\stm32f10x.h"

#line 4233 "user\\stm32f10x.h"

 
#line 4244 "user\\stm32f10x.h"

#line 4254 "user\\stm32f10x.h"

#line 4264 "user\\stm32f10x.h"

#line 4274 "user\\stm32f10x.h"

 
#line 4285 "user\\stm32f10x.h"

#line 4295 "user\\stm32f10x.h"

#line 4305 "user\\stm32f10x.h"

#line 4315 "user\\stm32f10x.h"

 
#line 4326 "user\\stm32f10x.h"

#line 4336 "user\\stm32f10x.h"

#line 4346 "user\\stm32f10x.h"

#line 4356 "user\\stm32f10x.h"

 
#line 4367 "user\\stm32f10x.h"

#line 4377 "user\\stm32f10x.h"

#line 4387 "user\\stm32f10x.h"

#line 4397 "user\\stm32f10x.h"

 
#line 4408 "user\\stm32f10x.h"

#line 4418 "user\\stm32f10x.h"

#line 4428 "user\\stm32f10x.h"

#line 4438 "user\\stm32f10x.h"

 
#line 4449 "user\\stm32f10x.h"

#line 4459 "user\\stm32f10x.h"

#line 4469 "user\\stm32f10x.h"

#line 4479 "user\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






#line 4527 "user\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 4597 "user\\stm32f10x.h"

 
#line 4612 "user\\stm32f10x.h"

 
#line 4638 "user\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
#line 4859 "user\\stm32f10x.h"

 
#line 4871 "user\\stm32f10x.h"

 






 
#line 4888 "user\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


#line 5032 "user\\stm32f10x.h"



 


#line 5044 "user\\stm32f10x.h"



 


#line 5056 "user\\stm32f10x.h"



 


#line 5068 "user\\stm32f10x.h"



 


#line 5080 "user\\stm32f10x.h"



 


#line 5092 "user\\stm32f10x.h"



 


#line 5104 "user\\stm32f10x.h"



 


#line 5116 "user\\stm32f10x.h"



 

 


#line 5130 "user\\stm32f10x.h"



 


#line 5142 "user\\stm32f10x.h"



 


#line 5154 "user\\stm32f10x.h"



 


#line 5166 "user\\stm32f10x.h"



 


#line 5178 "user\\stm32f10x.h"



 


#line 5190 "user\\stm32f10x.h"



 


#line 5202 "user\\stm32f10x.h"



 


#line 5214 "user\\stm32f10x.h"



 


#line 5226 "user\\stm32f10x.h"



 


#line 5238 "user\\stm32f10x.h"



 


#line 5250 "user\\stm32f10x.h"



 


#line 5262 "user\\stm32f10x.h"



 


#line 5274 "user\\stm32f10x.h"



 


#line 5286 "user\\stm32f10x.h"



 


#line 5298 "user\\stm32f10x.h"



 


#line 5310 "user\\stm32f10x.h"



 
 
 
 
 

 
 
#line 5330 "user\\stm32f10x.h"

 
#line 5341 "user\\stm32f10x.h"

 
#line 5359 "user\\stm32f10x.h"











 





 





 
#line 5397 "user\\stm32f10x.h"

 












 
#line 5418 "user\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 5558 "user\\stm32f10x.h"

 
#line 5575 "user\\stm32f10x.h"

 
#line 5592 "user\\stm32f10x.h"

 
#line 5609 "user\\stm32f10x.h"

 
#line 5643 "user\\stm32f10x.h"

 
#line 5677 "user\\stm32f10x.h"

 
#line 5711 "user\\stm32f10x.h"

 
#line 5745 "user\\stm32f10x.h"

 
#line 5779 "user\\stm32f10x.h"

 
#line 5813 "user\\stm32f10x.h"

 
#line 5847 "user\\stm32f10x.h"

 
#line 5881 "user\\stm32f10x.h"

 
#line 5915 "user\\stm32f10x.h"

 
#line 5949 "user\\stm32f10x.h"

 
#line 5983 "user\\stm32f10x.h"

 
#line 6017 "user\\stm32f10x.h"

 
#line 6051 "user\\stm32f10x.h"

 
#line 6085 "user\\stm32f10x.h"

 
#line 6119 "user\\stm32f10x.h"

 
#line 6153 "user\\stm32f10x.h"

 
#line 6187 "user\\stm32f10x.h"

 
#line 6221 "user\\stm32f10x.h"

 
#line 6255 "user\\stm32f10x.h"

 
#line 6289 "user\\stm32f10x.h"

 
#line 6323 "user\\stm32f10x.h"

 
#line 6357 "user\\stm32f10x.h"

 
#line 6391 "user\\stm32f10x.h"

 
#line 6425 "user\\stm32f10x.h"

 
#line 6459 "user\\stm32f10x.h"

 
#line 6493 "user\\stm32f10x.h"

 
#line 6527 "user\\stm32f10x.h"

 
#line 6561 "user\\stm32f10x.h"

 
 
 
 
 

 









#line 6588 "user\\stm32f10x.h"

 
#line 6596 "user\\stm32f10x.h"

 
#line 6606 "user\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
#line 6667 "user\\stm32f10x.h"

 
#line 6676 "user\\stm32f10x.h"







 



#line 6697 "user\\stm32f10x.h"



 



 


 
#line 6722 "user\\stm32f10x.h"

 
#line 6732 "user\\stm32f10x.h"

 




 


 
 
 
 
 

 
#line 6758 "user\\stm32f10x.h"

 


 



 
#line 6781 "user\\stm32f10x.h"

 
#line 6790 "user\\stm32f10x.h"







 
#line 6809 "user\\stm32f10x.h"

 
#line 6820 "user\\stm32f10x.h"



 
 
 
 
 

 


#line 6849 "user\\stm32f10x.h"

 









#line 6873 "user\\stm32f10x.h"

 
 
 
 
 

 









 


 


 





 
#line 6913 "user\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 





 

 

  

#line 1 ".\\FWlib\\inc\\stm32f10x_conf.h"


















 

 



 
 
#line 1 ".\\FWlib\\inc\\stm32f10x_adc.h"



















  

 



 
#line 1 ".\\user\\stm32f10x.h"





















 



 



 
 
#line 6999 ".\\user\\stm32f10x.h"



 

  

 

 
#line 28 ".\\FWlib\\inc\\stm32f10x_adc.h"
#line 1 ".\\FWlib\\inc\\stm32f10x_conf.h"


















 

 
#line 75 ".\\FWlib\\inc\\stm32f10x_conf.h"

 
#line 29 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 



 



 

typedef struct
{
  uint32_t ADC_Mode;
  FunctionalState ADC_ScanConvMode; 
  FunctionalState ADC_ContinuousConvMode;
  uint32_t ADC_ExternalTrigConv;
  uint32_t ADC_DataAlign;
  uint8_t ADC_NbrOfChannel;
}ADC_InitTypeDef;


 



 










 

#line 83 ".\\FWlib\\inc\\stm32f10x_adc.h"

#line 94 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 



 

#line 112 ".\\FWlib\\inc\\stm32f10x_adc.h"



 






 

#line 130 ".\\FWlib\\inc\\stm32f10x_adc.h"

#line 145 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 







 



 

#line 183 ".\\FWlib\\inc\\stm32f10x_adc.h"

#line 193 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 217 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 



 









 







 







#line 266 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 282 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 297 ".\\FWlib\\inc\\stm32f10x_adc.h"

#line 305 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 











 



 

#line 338 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 





 



 





 



 





 



 





  




 




 



 





 



 





 



 



 



 



 

void ADC_DeInit(ADC_TypeDef* ADCx);
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);
void ADC_ResetCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_StartCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);
uint32_t ADC_GetDualModeConversionValue(void);
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv);
void ADC_ExternalTrigInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx);
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length);
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset);
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel);
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold, uint16_t LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel);
void ADC_TempSensorVrefintCmd(FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);





 



 



 

 
#line 28 ".\\FWlib\\inc\\stm32f10x_conf.h"
 
#line 1 ".\\FWlib\\inc\\stm32f10x_can.h"



















  

 



 
#line 28 ".\\FWlib\\inc\\stm32f10x_can.h"



 



 



 





 

typedef struct
{
  FunctionalState CAN_TTCM;
  FunctionalState CAN_ABOM;
  FunctionalState CAN_AWUM;
  FunctionalState CAN_NART;
  FunctionalState CAN_RFLM;
  FunctionalState CAN_TXFP;
  uint8_t CAN_Mode;
  uint8_t CAN_SJW;
  uint8_t CAN_BS1;
  uint8_t CAN_BS2;
  uint16_t CAN_Prescaler;
} CAN_InitTypeDef;



 

typedef struct
{
  uint8_t CAN_FilterNumber;
  uint8_t CAN_FilterMode;
  uint8_t CAN_FilterScale;
  uint16_t CAN_FilterIdHigh;
  uint16_t CAN_FilterIdLow;
  uint16_t CAN_FilterMaskIdHigh;
  uint16_t CAN_FilterMaskIdLow;
  uint16_t CAN_FilterFIFOAssignment;
  FunctionalState CAN_FilterActivation;
} CAN_FilterInitTypeDef;



 

typedef struct
{
  uint32_t StdId;
  uint32_t ExtId;
  uint8_t IDE;
  uint8_t RTR;
  uint8_t DLC;
  uint8_t Data[8];
} CanTxMsg;



 

typedef struct
{
  uint32_t StdId;
  uint32_t ExtId;
  uint8_t IDE;
  uint8_t RTR;
  uint8_t DLC;
  uint8_t Data[8];
  uint8_t FMI;
} CanRxMsg;



 



 



 






 



 










 



 










 



 

#line 177 ".\\FWlib\\inc\\stm32f10x_can.h"




 



 

#line 195 ".\\FWlib\\inc\\stm32f10x_can.h"





 



 





 



 





 



 








 



 









 



 








 



 








 



 







 



 







 



 








 



 








 



 






 



 






 



 










 



 

#line 383 ".\\FWlib\\inc\\stm32f10x_can.h"

#line 391 ".\\FWlib\\inc\\stm32f10x_can.h"

#line 398 ".\\FWlib\\inc\\stm32f10x_can.h"


 



 



 



 



 

void CAN_DeInit(CAN_TypeDef* CANx);
uint8_t CAN_Init(CAN_TypeDef* CANx, CAN_InitTypeDef* CAN_InitStruct);
void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct);
void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct);
void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState);
uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);
uint8_t CAN_TransmitStatus(CAN_TypeDef* CANx, uint8_t TransmitMailbox);
void CAN_CancelTransmit(CAN_TypeDef* CANx, uint8_t Mailbox);
void CAN_FIFORelease(CAN_TypeDef* CANx, uint8_t FIFONumber);
uint8_t CAN_MessagePending(CAN_TypeDef* CANx, uint8_t FIFONumber);
void CAN_Receive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage);
void CAN_DBGFreeze(CAN_TypeDef* CANx, FunctionalState NewState);
uint8_t CAN_Sleep(CAN_TypeDef* CANx);
uint8_t CAN_WakeUp(CAN_TypeDef* CANx);
FlagStatus CAN_GetFlagStatus(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
void CAN_ClearFlag(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
ITStatus CAN_GetITStatus(CAN_TypeDef* CANx, uint32_t CAN_IT);
void CAN_ClearITPendingBit(CAN_TypeDef* CANx, uint32_t CAN_IT);




 



 



 

 
#line 30 ".\\FWlib\\inc\\stm32f10x_conf.h"
 
 
 
#line 1 ".\\FWlib\\inc\\stm32f10x_dma.h"



















  

 



 
#line 28 ".\\FWlib\\inc\\stm32f10x_dma.h"



 



 



 



 

typedef struct
{
  uint32_t DMA_PeripheralBaseAddr;
  uint32_t DMA_MemoryBaseAddr;
  uint32_t DMA_DIR;
  uint32_t DMA_BufferSize;
  uint32_t DMA_PeripheralInc;
  uint32_t DMA_MemoryInc;
  uint32_t DMA_PeripheralDataSize;
  uint32_t DMA_MemoryDataSize;
  uint32_t DMA_Mode;
  uint32_t DMA_Priority;
  uint32_t DMA_M2M;
}DMA_InitTypeDef;



 



 

#line 80 ".\\FWlib\\inc\\stm32f10x_dma.h"



 







 



 







 



 







 



 

#line 127 ".\\FWlib\\inc\\stm32f10x_dma.h"


 



 

#line 141 ".\\FWlib\\inc\\stm32f10x_dma.h"


 



 






 



 

#line 168 ".\\FWlib\\inc\\stm32f10x_dma.h"


 



 







 



 








 

#line 225 ".\\FWlib\\inc\\stm32f10x_dma.h"



 

#line 250 ".\\FWlib\\inc\\stm32f10x_dma.h"



#line 277 ".\\FWlib\\inc\\stm32f10x_dma.h"



 



 



 

#line 318 ".\\FWlib\\inc\\stm32f10x_dma.h"



 

#line 343 ".\\FWlib\\inc\\stm32f10x_dma.h"



#line 370 ".\\FWlib\\inc\\stm32f10x_dma.h"


 



 





 



 



 



 



 

void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx);
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Channel_TypeDef* DMAy_Channelx, FunctionalState NewState);
void DMA_ITConfig(DMA_Channel_TypeDef* DMAy_Channelx, uint32_t DMA_IT, FunctionalState NewState);
uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);
FlagStatus DMA_GetFlagStatus(uint32_t DMA_FLAG);
void DMA_ClearFlag(uint32_t DMA_FLAG);
ITStatus DMA_GetITStatus(uint32_t DMA_IT);
void DMA_ClearITPendingBit(uint32_t DMA_IT);




 



 



 

 
#line 34 ".\\FWlib\\inc\\stm32f10x_conf.h"
 
#line 1 ".\\FWlib\\inc\\stm32f10x_flash.h"



















  

 



 
#line 28 ".\\FWlib\\inc\\stm32f10x_flash.h"



 



 



 



 

typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;



 



 



 

#line 72 ".\\FWlib\\inc\\stm32f10x_flash.h"


 



 







 



 







 



 


 
#line 138 ".\\FWlib\\inc\\stm32f10x_flash.h"


 
#line 174 ".\\FWlib\\inc\\stm32f10x_flash.h"









 



 







 



 







 



 







 



 







 



 






 







 



 



 



 



 

void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_HalfCycleAccessCmd(uint32_t FLASH_HalfCycleAccess);
void FLASH_PrefetchBufferCmd(uint32_t FLASH_PrefetchBuffer);
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_EraseOptionBytes(void);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data);
FLASH_Status FLASH_EnableWriteProtection(uint32_t FLASH_Pages);
FLASH_Status FLASH_ReadOutProtection(FunctionalState NewState);
FLASH_Status FLASH_UserOptionByteConfig(uint16_t OB_IWDG, uint16_t OB_STOP, uint16_t OB_STDBY);
uint32_t FLASH_GetUserOptionByte(void);
uint32_t FLASH_GetWriteProtectionOptionByte(void);
FlagStatus FLASH_GetReadOutProtectionStatus(void);
FlagStatus FLASH_GetPrefetchBufferStatus(void);
void FLASH_ITConfig(uint16_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint16_t FLASH_FLAG);
void FLASH_ClearFlag(uint16_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout);




 



 



 

 
#line 36 ".\\FWlib\\inc\\stm32f10x_conf.h"
 
#line 1 ".\\FWlib\\inc\\stm32f10x_gpio.h"



















  

 



 
#line 28 ".\\FWlib\\inc\\stm32f10x_gpio.h"



 



 



 

#line 48 ".\\FWlib\\inc\\stm32f10x_gpio.h"
                                     


 

typedef enum
{ 
  GPIO_Speed_10MHz = 1,
  GPIO_Speed_2MHz, 
  GPIO_Speed_50MHz
}GPIOSpeed_TypeDef;





 

typedef enum
{ GPIO_Mode_AIN = 0x0,		       
  GPIO_Mode_IN_FLOATING = 0x04,	   
  GPIO_Mode_IPD = 0x28,	           
  GPIO_Mode_IPU = 0x48,			   
  GPIO_Mode_Out_OD = 0x14,		   
  GPIO_Mode_Out_PP = 0x10,		   
  GPIO_Mode_AF_OD = 0x1C,		   
  GPIO_Mode_AF_PP = 0x18		   
}GPIOMode_TypeDef;








 

typedef struct
{
  uint16_t GPIO_Pin;
  GPIOSpeed_TypeDef GPIO_Speed;
  GPIOMode_TypeDef GPIO_Mode;
}GPIO_InitTypeDef;



 

typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;





 



 



 

#line 133 ".\\FWlib\\inc\\stm32f10x_gpio.h"



#line 152 ".\\FWlib\\inc\\stm32f10x_gpio.h"



 



 

#line 186 ".\\FWlib\\inc\\stm32f10x_gpio.h"

#line 200 ".\\FWlib\\inc\\stm32f10x_gpio.h"
                              


  



 

#line 221 ".\\FWlib\\inc\\stm32f10x_gpio.h"

#line 229 ".\\FWlib\\inc\\stm32f10x_gpio.h"



 



 

#line 254 ".\\FWlib\\inc\\stm32f10x_gpio.h"

#line 271 ".\\FWlib\\inc\\stm32f10x_gpio.h"



 



 



 



 



 

void GPIO_DeInit(GPIO_TypeDef* GPIOx);
void GPIO_AFIODeInit(void);
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_EventOutputCmd(FunctionalState NewState);
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);




 



 



 

 
#line 38 ".\\FWlib\\inc\\stm32f10x_conf.h"
 
 
 
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




 



 



  

 
#line 42 ".\\FWlib\\inc\\stm32f10x_conf.h"
 
 
 
#line 1 ".\\FWlib\\inc\\stm32f10x_tim.h"



















  

 



 
#line 28 ".\\FWlib\\inc\\stm32f10x_tim.h"



 



  



  



 

typedef struct
{
  uint16_t TIM_Prescaler;
  uint16_t TIM_CounterMode;
  uint16_t TIM_Period;
  uint16_t TIM_ClockDivision;
  uint8_t TIM_RepetitionCounter;
} TIM_TimeBaseInitTypeDef;



 

typedef struct
{
  uint16_t TIM_OCMode;
  uint16_t TIM_OutputState;
  uint16_t TIM_OutputNState;
  uint16_t TIM_Pulse;
  uint16_t TIM_OCPolarity;
  uint16_t TIM_OCNPolarity;
  uint16_t TIM_OCIdleState;
  uint16_t TIM_OCNIdleState;
} TIM_OCInitTypeDef;



 

typedef struct
{
  uint16_t TIM_Channel;
  uint16_t TIM_ICPolarity;
  uint16_t TIM_ICSelection;
  uint16_t TIM_ICPrescaler;
  uint16_t TIM_ICFilter;
} TIM_ICInitTypeDef;



 

typedef struct
{
  uint16_t TIM_OSSRState;
  uint16_t TIM_OSSIState;
  uint16_t TIM_LOCKLevel; 
  uint16_t TIM_DeadTime;
  uint16_t TIM_Break;
  uint16_t TIM_BreakPolarity;
  uint16_t TIM_AutomaticOutput;
} TIM_BDTRInitTypeDef;



 

#line 118 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 146 ".\\FWlib\\inc\\stm32f10x_tim.h"


 



 







  



 

#line 179 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 193 ".\\FWlib\\inc\\stm32f10x_tim.h"


 



 

#line 211 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 







 



 
  






 



 







  



 







  



 







  



 







  



 







  



 







  



 







  



 

#line 335 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 







 



 







  



 







  



 







  



 







  



 

#line 409 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 425 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 457 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 503 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 547 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 570 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 586 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 614 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 628 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



  






 



 







  



 







  



 

#line 677 ".\\FWlib\\inc\\stm32f10x_tim.h"


  




 

#line 702 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 







  



 







  



 





                                     


  



 







  



 

#line 810 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 826 ".\\FWlib\\inc\\stm32f10x_tim.h"


  



 







  



 

#line 895 ".\\FWlib\\inc\\stm32f10x_tim.h"



  



 




  



 




  



 



 



  



 

void TIM_DeInit(TIM_TypeDef* TIMx);
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct);
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource);
void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter);
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                             uint16_t ExtTRGFilter);
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
                             uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                   uint16_t ExtTRGFilter);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode);
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN);
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode);
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);
void TIM_SetCounter(TIM_TypeDef* TIMx, uint16_t Counter);
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint16_t Autoreload);
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint16_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint16_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint16_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint16_t Compare4);
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD);
uint16_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture4(TIM_TypeDef* TIMx);
uint16_t TIM_GetCounter(TIM_TypeDef* TIMx);
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);




  



  



 

 
#line 46 ".\\FWlib\\inc\\stm32f10x_conf.h"
#line 1 ".\\FWlib\\inc\\stm32f10x_usart.h"



















  

 



 
#line 28 ".\\FWlib\\inc\\stm32f10x_usart.h"



 



  



  



  
  
typedef struct
{
  uint32_t USART_BaudRate;
  uint16_t USART_WordLength;
  uint16_t USART_StopBits;
  uint16_t USART_Parity;
  uint16_t USART_Mode;
  uint16_t USART_HardwareFlowControl;  
} USART_InitTypeDef;



  
  
typedef struct
{
  uint16_t USART_Clock;
  uint16_t USART_CPOL;
  uint16_t USART_CPHA;
  uint16_t USART_LastBit;
} USART_ClockInitTypeDef;



  



  
  
#line 87 ".\\FWlib\\inc\\stm32f10x_usart.h"


  
  


                                    




  



  
  
#line 112 ".\\FWlib\\inc\\stm32f10x_usart.h"


  



  
  
#line 126 ".\\FWlib\\inc\\stm32f10x_usart.h"


  



  
  





  



  
#line 156 ".\\FWlib\\inc\\stm32f10x_usart.h"


  



  






  



 
  






  



 







 



 







  



 
  
#line 236 ".\\FWlib\\inc\\stm32f10x_usart.h"


 



 







  



 







 



 
  







 



 







  



 

#line 308 ".\\FWlib\\inc\\stm32f10x_usart.h"
                              
#line 316 ".\\FWlib\\inc\\stm32f10x_usart.h"



  



  



  



  



 

void USART_DeInit(USART_TypeDef* USARTx);
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);
void USART_SendBreak(USART_TypeDef* USARTx);
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);




  



  



  

 
#line 47 ".\\FWlib\\inc\\stm32f10x_conf.h"
 
#line 1 ".\\FWlib\\inc\\misc.h"



















  

 



 
#line 28 ".\\FWlib\\inc\\misc.h"



 



 



 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;
  uint8_t NVIC_IRQChannelPreemptionPriority;
  uint8_t NVIC_IRQChannelSubPriority;
  FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;



 



 



 







 



 

#line 83 ".\\FWlib\\inc\\misc.h"


 



 

#line 101 ".\\FWlib\\inc\\misc.h"















 



 







 



 



 



 



 

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);





 



 



 

 
#line 49 ".\\FWlib\\inc\\stm32f10x_conf.h"

 
 

 
 

 
#line 73 ".\\FWlib\\inc\\stm32f10x_conf.h"



 
#line 6974 "user\\stm32f10x.h"




 

















 





 

  

 

 
#line 6 "user\\debugger.h"
#line 7 "user\\debugger.h"
#line 8 "user\\debugger.h"

#line 1 "user\\main.h"



#line 5 "user\\main.h"
#line 6 "user\\main.h"



#line 1 "user\\motion.h"



#line 5 "user\\motion.h"





void motion_set_motor(s32 ,u8);


#line 10 "user\\main.h"
#line 1 "user\\system.h"



#line 5 "user\\system.h"
#line 6 "user\\system.h"
#line 7 "user\\system.h"
#line 8 "user\\system.h"
#line 9 "user\\system.h"
#line 10 "user\\system.h"

void system_init(u16 sysFreq);
void system_pwm_enable(void);
void system_para_init(void);





#line 11 "user\\main.h"
#line 1 "user\\pid_vel.h"



#line 5 "user\\pid_vel.h"
#line 6 "user\\pid_vel.h"





#line 12 "user\\pid_vel.h"
#line 1 "user\\pid_pos.h"



#line 5 "user\\pid_pos.h"
#line 6 "user\\pid_pos.h"





#line 12 "user\\pid_pos.h"
#line 1 "user\\pid_vel.h"
#line 13 "user\\pid_pos.h"

float abs(float);

void pos_set_pid(float _p, float _i, float _d);
void pos_set_max(float _acc, float _count);
float pos_get_curr(void);
void pos_set_home(void);

void pos_err(void);

void pos_cal_function_mode(void);
void pos_move_function_mode(u8);

void pos_move(u8);

void motor_set_position(s32 position, u16 vel);

#line 13 "user\\pid_vel.h"







void vel_set_pid1(float _p, float _i, float _d);
void vel_set_pid2(float _p, float _i, float _d);
void vel_init_pwm(float _pwm, s8 _dir);
void vel_set_max(float m);
void vel_set_speed(float r);
float vel_get_speed(void);


void clear_record(void);
void init_memory(void);
void vel_err(void);
void set_default_pid(void);
void set_pwm_to_motor(void);
void read_encoder(void);
void get_current(void);


void vel_move(u8);
void vel_stop(u8);
void vel_zero(void);


void vel_n_pos(void);


void motor_set_pwm(float user_pwm);
void motor_set_pwm_current(float user_pwm);
void motor_set_speed(float speed);
void motor_lock(void);
void motor_zero(void);
void cali_user(void);
void set_acceleration(u16 value);
void increase_encoder(void);

#line 12 "user\\main.h"
#line 13 "user\\main.h"
#line 1 "user\\debugger.h"
#line 14 "user\\main.h"
#line 1 "user\\Robocon_CAN.h"
 



 
#line 7 "user\\Robocon_CAN.h"
#line 8 "user\\Robocon_CAN.h"
#line 9 "user\\Robocon_CAN.h"
#line 10 "user\\Robocon_CAN.h"













 






 



void CAN_Configuration(void);
void test_tx(void);
void CAN_addToQueue(CanRxMsg RxMsg);
u8 CAN_dequeue(void);
void CAN_messageProcessing(CanRxMsg RxMsg);
void USB_LP_CAN_RX0_IRQHandler(void);
extern u8 can_rx_data;
extern CanRxMsg CAN_queueHead;
extern s32 CAN_QueueCounter;
extern CanRxMsg CAN_Queue[256];

 
void Main_TX(void);
void Device1_TX(void);
void Device2_TX(void);
void Device3_TX(void);
void Device4_TX(void);

 





 





#line 80 "user\\Robocon_CAN.h"



 
extern u8 Device1_data;
extern u8 Device2_data;
extern u8 Device3_data;
extern u8 Device4_data;

#line 15 "user\\main.h"
#line 1 "user\\Robocon_CANProtocol.h"



#line 5 "user\\Robocon_CANProtocol.h"
#line 6 "user\\Robocon_CANProtocol.h"
#line 7 "user\\Robocon_CANProtocol.h"










 

 












 






 





















 







 
extern s32 Encoder_Count[16];			
extern s16 Robot_Coordinate_X;						
extern s16 Robot_Coordinate_Y;
extern s16 Robot_Angle;
extern s16 Robot_Velocity_X;						
extern s16 Robot_Velocity_Y;
extern s16 Robot_Angular_Velocity;

extern u32 receive_counter;	   		
extern s32 test_motor_vel;
extern u8 test_motor_flag;
extern u16 test_motor_vel_mag;
extern s32 test_motor_pos;	   

extern u8 Tx_QueueCounter;
extern u8 Rx_QueueCounter;

extern s32 test_data1;
extern s32 test_data2;
extern s32 test_data3;
extern s32 test_data4;	

 
typedef struct
{
	u8 Data_Length;
	u8 Data[8];									
} Data_Field;

 
s32 Four_Bytes_Reconstruction(const uint8_t* buffer);
s16 Two_Bytes_Reconstruction(const uint8_t* buffer);
u8 Four_Bytes_Decomposition(const s32 data, const u8 index);
u8 Two_Bytes_Decomposition(const u16 data, const u8 index);

CanTxMsg General_Encoding(u32 Device_ID, Data_Field Cmd_Data);
CanTxMsg Motor_Velocity_Encoding(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag);
CanTxMsg Motor_Position_Encoding(u32 Motor_ID, u16 vel, s32 pos);

void Motor_Feedback_Decoding(CanRxMsg RxMessage);
void Gyroscope_Feedback_Decoding(CanRxMsg RxMessage);

void CAN_Tx_addToQueue(CanTxMsg TxMessage);
u8 CAN_Tx_dequeue(void);

void CAN_Rx_addToQueue(CanRxMsg RxMessage);
void CAN_Rx_Processing(CanRxMsg RxMessage);
u8 CAN_Rx_dequeue(void);

void CAN_Tx_update(void);
void CAN_Rx_update(void);

void USB_LP_CAN1_RX0_IRQHandler(void);

void Motor_Set_Vel(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag);
void Motor_Set_Pos(u32 Motor_ID, u16 vel, s32 pos);
void Send_Encoder(s32 Data);
void Calibration_Done(void);

void Motor_Set_Test(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag);

void debug_data1(s32 data);
void debug_data2(s32 data);
void debug_data3(s32 data);
void debug_data4(s32 data);





#line 16 "user\\main.h"
#line 1 "user\\ticks.h"



#line 5 "user\\ticks.h"
#line 6 "user\\ticks.h"
#line 7 "user\\ticks.h"

extern volatile u16 main_ticks;
extern volatile u16 seconds;

void ticks_init(void);
u16 get_ticks(void);
u16 get_seconds(void);

void TIM4_IRQHandler(void);
#line 17 "user\\main.h"















 




#line 44 "user\\main.h"







#line 64 "user\\main.h"

















 




#line 92 "user\\main.h"



#line 10 "user\\debugger.h"

void system_shake_hand (u8 shake);






 
void update_token(void);
void send_debug_msg(u8 index, float data);
void send_float(u8 cmd, float* data);
void send_3_floats(u8 cmd, float* data1, float* data2, float* data3);
void debugger_msg(void);





#line 2 "user\\debugger.c"


extern u32 ticks;


extern float pwm;
extern u8 start;
u8 _start;
extern u8 mode;
extern u8 dir;
u8 _dir;

extern float d_count;
extern float r_count;
extern u16 cmd;
extern u8 check;

extern u8 connect_state;

extern u8 tx_buffer[];
extern u8 tx_buffer_token;
extern u8 tx_data_size;
float				float32[6];
unsigned char		conversion[4];




void update_token(void)
{
    tx_buffer_token++;
	
	if (tx_buffer_token>200-1)
	    tx_buffer_token=0;
}

void system_shake_hand (u8 shake)
{
	connect_state = shake;
	tx_buffer[tx_buffer_token]=shake;
	update_token();
	tx_buffer[tx_buffer_token]=0xfe;
	update_token();
	tx_buffer[tx_buffer_token]=0xff;
	update_token();
    tx_data_size=3;
	USART_ITConfig(((USART_TypeDef *) (((uint32_t)0x40000000) + 0x4800)), ((uint16_t)0x0626), ENABLE);

}


void send_debug_msg(u8 index, float data)
{
    u8 *tmp = (u8 *) &data;
	tx_buffer[tx_buffer_token]=0x31 + index;
	update_token();

	tx_buffer[tx_buffer_token]=tmp[0];
	update_token();
	tx_buffer[tx_buffer_token]=tmp[1];
	update_token();
	tx_buffer[tx_buffer_token]=tmp[2];
	update_token();
	tx_buffer[tx_buffer_token]=tmp[3];
	update_token();

	tx_buffer[tx_buffer_token]=0xfe;
	update_token();
	tx_buffer[tx_buffer_token]=0xff;
	update_token();

    tx_data_size=7;
	USART_ITConfig(((USART_TypeDef *) (((uint32_t)0x40000000) + 0x4800)), ((uint16_t)0x0626), ENABLE);
}

void send_float(u8 cmd, float* data)
{
    u8 *tmp = (u8 *) data;
	tx_buffer[tx_buffer_token]=cmd;
	update_token();

	tx_buffer[tx_buffer_token]=tmp[0];
	update_token();
	tx_buffer[tx_buffer_token]=tmp[1];
	update_token();
	tx_buffer[tx_buffer_token]=tmp[2];
	update_token();
	tx_buffer[tx_buffer_token]=tmp[3];
	update_token();

	tx_buffer[tx_buffer_token]=0xfe;
	update_token();
	tx_buffer[tx_buffer_token]=0xff;
	update_token();

    tx_data_size=7;
	USART_ITConfig(((USART_TypeDef *) (((uint32_t)0x40000000) + 0x4800)), ((uint16_t)0x0626), ENABLE);
}

void send_3_floats(u8 cmd, float* data1, float* data2, float* data3)
{
    u8 *tmp1 = (u8 *) data1;
	u8 *tmp2 = (u8 *) data2;
	u8 *tmp3 = (u8 *) data3;

	tx_buffer[tx_buffer_token]=cmd;
	update_token();

	tx_buffer[tx_buffer_token]=tmp1[0];
	update_token();
	tx_buffer[tx_buffer_token]=tmp1[1];
	update_token();
	tx_buffer[tx_buffer_token]=tmp1[2];
	update_token();
	tx_buffer[tx_buffer_token]=tmp1[3];
	update_token();

	tx_buffer[tx_buffer_token]=tmp2[0];
	update_token();
	tx_buffer[tx_buffer_token]=tmp2[1];
	update_token();
	tx_buffer[tx_buffer_token]=tmp2[2];
	update_token();
	tx_buffer[tx_buffer_token]=tmp2[3];
	update_token();

	tx_buffer[tx_buffer_token]=tmp3[0];
	update_token();
	tx_buffer[tx_buffer_token]=tmp3[1];
	update_token();
	tx_buffer[tx_buffer_token]=tmp3[2];
	update_token();
	tx_buffer[tx_buffer_token]=tmp3[3];
	update_token();

	tx_buffer[tx_buffer_token]=0xfe;
	update_token();
	tx_buffer[tx_buffer_token]=0xff;
	update_token();

	tx_data_size=15;
	USART_ITConfig(((USART_TypeDef *) (((uint32_t)0x40000000) + 0x4800)), ((uint16_t)0x0626), ENABLE);
}

extern vu16 ADC_ConvertedValue[];
extern float count;
extern float clock;
u32 clock2;
extern float d_cal;
extern float d_carrier;
extern float ADC_offset[];
extern float current_input;
extern float PID;
extern float count_total;
extern float current_stream[];
extern float motor_current;
extern float ADC_offset[];

void debugger_msg(void)
{					 

	send_debug_msg(1, ADC_ConvertedValue[0]);
	send_debug_msg(2, ADC_ConvertedValue[1]);






	




 

}
