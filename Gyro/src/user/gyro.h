#ifndef __GYRO_H
#define __GYRO_H

#include "stm32f10x.h"
#include <stdio.h>
#include "delay.h"
#include "approx_math.h"
#include "stm32f10x_tim.h"

#define GYRO_SPI				SPI1
#define GYRO_CLK				RCC_APB2Periph_SPI1
#define GYRO_GPIO				GPIOA
#define GYRO_GPIO_CLK			RCC_APB2Periph_GPIOA
#define GYRO_GPIO_SPEED			GPIO_Speed_2MHz 

#define GYRO_PIN_NSS			GPIO_Pin_4
#define GYRO_PIN_SCK            GPIO_Pin_5
#define GYRO_PIN_MISO           GPIO_Pin_6
#define GYRO_PIN_MOSI           GPIO_Pin_7
#define GYRO_MODE				SPI_Mode_Master				
#define GYRO_BR_Prescaler		SPI_BaudRatePrescaler_64


#define GYRO_ANG_VEL_TH 		10
#define GYRO_SCALE 				350.4661806	//		1 / (0.07326 * 3.908ms ) / 10 =349.2838703

#define GYRO_FLASH				0x01
#define GYRO_POWER				0x03
#define GYRO_VEL				0x05
#define GYRO_ADC				0x0B
#define GYRO_TEMP				0x0D
#define GYRO_ANGL				0x0F
#define GYRO_OFF				0x15   
#define GYRO_COMD				0x3F
#define GYRO_SENS				0x39
#define GYRO_SMPL				0x37
	

#define X 0
#define Y 1

extern volatile s16 prev_ang_vel;
extern volatile s16 curr_ang_vel;
extern volatile s32 gyro_angle;
extern volatile s32 sim_now;
extern volatile s32 sim_angle;
extern volatile s32 gyro_comb;
extern volatile s32 gyro_temp;
extern volatile s16 real_angle;
extern u8 gyro_state;
//private functions
void gyro_spi_init(void);			   	//init spi communication channel
u16 spi_frame(u16 data);	   			//transmit 16-bit data via spi

void adis_write(u8 addr, u16 data);		//write data to gyro
u16 adis_read(u8 addr );				//read data from gyro

void gyro_chip_select( void );			//select chip
void gyro_chip_deselect( void );		//deselect chip


//public functions
void gyro_init(void);					//init gyro
void gyro_cal(void);					//cal gyro, only for first calibration
void gyro_cal_short(void);				//cal gyro in shorter time, for re-calibration 	 
void gyro_update_tim_init(void); 		//init timer for updating gyro				
void gyro_update(void);				//carry out the calculation		
														   	 
u8 gyro_get_state(void);				//read status for gyro
s16 gyro_get_vel(void);				//read angular velocity from gyro
u16 gyro_get_angle(void);				//read angle from gyro

s16 gyro_get_off(void);				//read offset(result of callibration) for angular velocity 
u16 gyro_get_flash(void);				//read number of flash for the rom un gyro
u16 gyro_get_power(void);	 			//return milli-volt
u16 gyro_get_adc(void);		  		//return milli-volt
u16 gyro_get_temp(void); 		   		//return milli-degree

void set_angle( s16 angle );

#endif	/* __GYRO_H */
