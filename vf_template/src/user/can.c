#include "can.h"

u8 CAN_FilterCount = 0;

void can_gpio_init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
			
	RCC_APB2PeriphClockCmd(CAN_RCC, ENABLE);
	
	// CAN_Rx Pin
	GPIO_InitStructure.GPIO_Pin = CAN_Rx_GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN_PORT, &GPIO_InitStructure);
	
	// CAN_Tx Pin
	GPIO_InitStructure.GPIO_Pin = CAN_Tx_GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN_PORT, &GPIO_InitStructure);
	
	
	
}

void can_nvic_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*
	#ifdef  VECT_TAB_RAM  
	// Set the Vector Table base location at 0x20000000 
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  // VECT_TAB_FLASH  
	// Set the Vector Table base location at 0x08000000  
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif
	*/

	/* enabling interrupt */
	NVIC_InitStructure.NVIC_IRQChannel=USB_LP_CAN1_RX0_IRQn; 

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
void can_init(void)
{
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
	/* GPIO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	can_gpio_init();
	can_nvic_init();

	
	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	
	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	
	/* CAN Baudrate = 1 MBPS */
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 6;
	CAN_Init(CAN1, &CAN_InitStructure);

	/* CAN FIFO0 message pending interrupt enable */ 
  	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	
	/* CAN filter init */

	/* 0xA0~0xAF */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0A0 << 5;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFE1F;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

}

/**
	* @brief Add filter to can data received (involves bitwise calculation)
	* @param id: 11-bit ID (0x000 to 0x7FF)
	* @param mask: 11-bit mask, corresponding to the 11-bit ID				
	* @example can_rx_add_filter(0x00, 0x00) will receive CAN message with ANY ID
	* @example can_rx_add_filter(0xCD, 0xFF) will receive CAN message with ID 0xCD
	* @example can_rx_add_filter(0xA0, 0xF0) will receive CAN message with ID from 0xA0 to 0xAF
	* @example can_rx_add_filter(0x00, 0xFA) will receive CAN message with ID from 0x00 to 0x03
	*/
void can_rx_add_filter(u16 id, u16 mask)
{
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
	CAN_FilterInitStructure.CAN_FilterNumber = CAN_FilterCount;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = (id << 5) & 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = ((mask << 5) | 0x001F) & 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);	
	
	++CAN_FilterCount;
}

/**
	* @brief Transfer a CAN message
	* @param msg: the CAN message
	* @retval True if the message is successfully tranferred
	*/

u8 can_tx(CanTxMsg msg)
{
	u8 Tx_MailBox = CAN_Transmit(CAN1, &msg);							//transmit the message
	u16 retry = CAN_TX_RETRY_TIMEOUT;
	while(CAN_TransmitStatus(CAN1,Tx_MailBox) != CANTXOK)	// Wait till transmission ends
	{
		if (--retry == 0) {
			return 0;
		}
	}
	return 1;
}
