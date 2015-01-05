/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CAN_H
#define _CAN_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "misc.h"

/**********************************************************************
*				Device Selection
*Description: Selecting the type of device by definition
*Explanation:
*Currently we have 5 types of devices: 
*1. main control 
*2. position system
*3. motor control
*4. sensor board
*5. sensor bar
*6. button board
*Select the target device by commenting the rest.
***********************************************************************/
#define MAIN_CONTROL
//#define POSITION_SYSTEM
//#define MOTOR_CONTROL
//#define SENSOR_BOARD
//#define SENSOR_BAR
//#define BUTTON_BOARD

#define STM_MD

/* Constants -----------------------------------------------------------------*/


#define CAN_QUEUE_LENGTH 255
#define CAN_QUEUE_EMPTY 0

#define	CAN_Rx_GPIO_Pin		GPIO_Pin_11
#define	CAN_Tx_GPIO_Pin		GPIO_Pin_12
#define	CAN_PORT					GPIOA
#define CAN_RCC						RCC_APB2Periph_GPIOA

void CAN_Configuration(void);
void test_tx(void);
void CAN_addToQueue(CanRxMsg RxMsg);
u8 CAN_dequeue(void);
void CAN_messageProcessing(CanRxMsg RxMsg);
#ifdef STM_HD 
void USB_LP_CAN_RX0_IRQHandler(void);
#else
void USB_LP_CAN1_RX0_IRQHandler(void);
#endif

extern u8 can_rx_data;
extern CanRxMsg CAN_queueHead;
extern s32 CAN_QueueCounter;
extern CanRxMsg CAN_Queue[CAN_QUEUE_LENGTH];

/* Devices ID definition --------------------------------*/

//Gyroscope(Position system)
#define POS_SYSTEM 					0x0A0
#define NEW_POS_SYSTEM			0x0A1
#define NEW_POS_SYSTEM_2			0x0A2

//Motors, Prepared 16 motors for future use, ID = 0x0BX
#define MOTOR1						0x0B0
#define MOTOR2						0x0B1
#define MOTOR3						0x0B2
#define MOTOR4						0x0B3
#define MOTOR5						0x0B4
#define MOTOR6						0x0B5
#define MOTOR7						0x0B6
#define MOTOR8						0x0B7
#define MOTOR9						0x0B8
#define MOTOR10						0x0B9
#define MOTOR11						0x0BA
#define MOTOR12						0x0BB
#define MOTOR13						0x0BC
#define MOTOR14						0x0BD
#define MOTOR15						0x0BE
#define MOTOR16						0x0BF

//Controller 
#define CONTROLLER				0x0C0

//Sensor Bar
#define SEN_BAR1					0x0C1
#define SEN_BAR2					0x0C2

//Sensor Board
#define SEN_BOARD					0x0D0

//Button Board
#define BTN_BOARD					0x0D1
#endif
