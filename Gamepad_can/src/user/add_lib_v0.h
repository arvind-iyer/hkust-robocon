#ifndef __ADD_LIB_V0_H
#define __ADD_LIB_V0_H
#include "main.h"

void zero(void);	//zero drive
void stop(void);	//stop abruptly
void manual_mode(void);
void io_test(void);

/************************************************************************************/
void test_menu(void);
//void parallel_test(void);
void gpio_g1_init(void);
void gpio_b_init(void);
void gpio_f_out_init(void);
void ns_delay(void);
void XBC_DATA_WRITE(u8 data);
void CH376_SET_PARALLEL_MOD(void);
u8 ch376_read_data(void);
u8 XBC_DATA_READ(void);
void ch376_set_parallel_mod(void);
void xbc_data_write(u8 data);
u8 xbc_data_read(void);


#define 	XBC_CTL_WR   	GPIO_Pin_7	//GPIOB
#define  	XBC_CTL_RD		GPIO_Pin_8 //GPIOB
#define 	XBC_CTL_PCS		GPIO_Pin_9	//GPIOB
#define 	XBC_CTL_AO 	 	GPIO_Pin_1	//GPIOG
#define     CH376_INT_N		GPIO_Pin_1  //GPIOE
#define     CH376_HW_RES	GPIO_Pin_0  //GPIOE

/*GPIOF*/
#define 	XBC_DATA_PORT	GPIOF
#define		XBC_DATA_D0	 	GPIO_Pin_1 
#define 	XBC_DATA_D1 	GPIO_Pin_2	
#define 	XBC_DATA_D2 	GPIO_Pin_3	
#define		XBC_DATA_D3   	GPIO_Pin_4  
#define 	XBC_DATA_D4 	GPIO_Pin_5    
#define 	XBC_DATA_D5  	GPIO_Pin_13	
#define  	XBC_DATA_D6  	GPIO_Pin_14   
#define  	XBC_DATA_D7  	GPIO_Pin_15   

/************************************************************************************/

#endif	/* __ADD_LIB_V0_H */		

