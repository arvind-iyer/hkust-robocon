#ifndef _CAN_H
#define _CAN_H

#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "misc.h"

#define	CANn							CAN1
#define CAN_RCC						RCC_APB1Periph_CAN1



#define	CAN_Rx_GPIO_Pin		GPIO_Pin_11
#define	CAN_Tx_GPIO_Pin		GPIO_Pin_12
#define	CAN_GPIO_PORT			GPIOA
#define CAN_GPIO_RCC			RCC_APB2Periph_GPIOA

#define CAN_TX_RETRY_TIMEOUT	800		// To prevent infinite loop

#define	CAN_RX_FILTER_LIMIT		28		// The number of filters can be applied at most

typedef enum {
	CAN_TX_FAILED 		= CAN_TxStatus_Failed,
	CAN_TX_OKAY				= CAN_TxStatus_Ok,
	CAN_TX_PENDING		= CAN_TxStatus_Pending,
	CAN_TX_NOMAILBOX	= CAN_TxStatus_NoMailBox
} CAN_TX_STATUS;

void can_init(void);
u8 can_tx(CanTxMsg msg);


#endif
