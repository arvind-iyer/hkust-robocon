/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CAN_H
#define _CAN_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "misc.h"

#include "configuration.h"

/* Constants -----------------------------------------------------------------*/
#define CAN_QUEUE_LENGTH 256
#define CAN_QUEUE_EMPTY 0

void CAN_Configuration(void);
void test_tx(void);
void CAN_addToQueue(CanRxMsg RxMsg);
u8 CAN_dequeue(void);
void CAN_messageProcessing(CanRxMsg RxMsg);
void USB_LP_CAN1_RX0_IRQHandler(void);
extern u8 can_rx_data;
extern CanRxMsg CAN_queueHead;
extern s32 CAN_QueueCounter;
extern CanRxMsg CAN_Queue[CAN_QUEUE_LENGTH];

/* Test ----------------------------------------------------------------------*/
void Device1_TX(void);
void Device2_TX(void);
void Device3_TX(void);
void Device4_TX(void);

/* Devices' IDs --------------------------------------------------------------*/
#define DEVICE_ONE 		0x321
#define DEVICE_TWO 		0x322
#define DEVICE_THREE 	0x323
#define DEVICE_FOUR 	0x324

/* Devices' data ---------------------------*/
extern u8 Device1_data;
extern u8 Device2_data;
extern u8 Device3_data;
extern u8 Device4_data;

#endif
