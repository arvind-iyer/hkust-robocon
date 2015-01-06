#ifndef _CAN_H
#define _CAN_H

#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "misc.h"


#define	CAN_Rx_GPIO_Pin		GPIO_Pin_11
#define	CAN_Tx_GPIO_Pin		GPIO_Pin_12
#define	CAN_PORT					GPIOA
#define CAN_RCC						RCC_APB2Periph_GPIOA

#define CAN_TX_RETRY_TIMEOUT	800		// To prevent infinite loop

void can_init(void);
void can_rx_add_filter(u16 id, u16 mask);
u8 can_tx(CanTxMsg msg);

void USB_LP_CAN1_RX0_IRQHandler(void);


/* Devices ID definition --------------------------------*/


#endif
