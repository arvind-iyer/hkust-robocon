/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CAN_H
#define _CAN_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "misc.h"
//#include "Robocon_config.h"

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
*Select the target device by commenting the rest.
***********************************************************************/
//#define MAIN_CONTROL
//#define POSITION_SYSTEM
#define MOTOR_CONTROL
//#define SENSOR_BORAD
//#define SENSOR_BAR	

/* Constants -----------------------------------------------------------------*/
#define CAN_QUEUE_LENGTH 256
#define CAN_QUEUE_EMPTY 0

void CAN_Configuration(void);
void test_tx(void);
void CAN_addToQueue(CanRxMsg RxMsg);
u8 CAN_dequeue(void);
void CAN_messageProcessing(CanRxMsg RxMsg);
void USB_LP_CAN_RX0_IRQHandler(void);
extern u8 can_rx_data;
extern CanRxMsg CAN_queueHead;
extern s32 CAN_QueueCounter;
extern CanRxMsg CAN_Queue[CAN_QUEUE_LENGTH];

/* Test ----------------------------------------------------------------------*/
void Main_TX(void);
void Device1_TX(void);
void Device2_TX(void);
void Device3_TX(void);
void Device4_TX(void);

/* Test Devices' IDs --------------------------------------------------------------*/
#define DEVICE_ONE 		0x321
#define DEVICE_TWO 		0x322
#define DEVICE_THREE 	0x323
#define DEVICE_FOUR 	0x324

/* Devices ID definition --------------------------------*/

//Gyroscope(Position system)
#define POS_SYSTEM 					0x0A0

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

#define MOTOR_ID 					MOTOR1

/* Devices' data ---------------------------*/
extern u8 Device1_data;
extern u8 Device2_data;
extern u8 Device3_data;
extern u8 Device4_data;

#endif
