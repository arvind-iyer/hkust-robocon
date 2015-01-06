#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include "ticks.h"
#include "can.h"


#define CAN_TX_QUEUE_SIZE 255
#define CAN_TX_DEQUEUE_TIMEOUT	1000
//#define CAN_RX_QUEUE_SIZE 100

#define	CAN_TX_TIM					TIM6
#define CAN_TX_RCC					RCC_APB1Periph_TIM6
#define	CAN_TX_IRQn					TIM6_IRQn
#define CAN_TX_IRQHander		void TIM6_IRQHandler(void)
	

typedef struct {
	u32 id;			/*** 11-bit ID: 0x000 to 0x7FF ***/
	u8 length;	/*** 0 to 8 ***/
	u8 data[8];
} CAN_MESSAGE;

typedef struct {
	u16 head;						/*** Current head of queue ***/
	u16 tail;						/*** Current tail of queue ***/
	const u16 length; 	/*** Length of queue ***/
	CAN_MESSAGE* queue;		/*** The can message queue (array) ***/
} CAN_QUEUE;

void can_tx_init(void);

u8 can_tx_enqueue(CAN_MESSAGE msg);
u8 can_tx_dequeue(void);

#endif /* __CAN_PROTOCOL_H */
