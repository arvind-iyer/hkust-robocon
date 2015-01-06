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

void CAN_AddFilter(u16 id, u16 mask);


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


#endif
