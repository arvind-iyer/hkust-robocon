#include "uart.h"

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

USART_TypeDef* COM_USART[COMn] = {USART1, USART2, USART3, UART4, UART5}; 
GPIO_TypeDef* COM_TX_PORT[COMn] = {COM1_TX_GPIO_PORT, COM2_TX_GPIO_PORT, COM3_TX_GPIO_PORT, COM4_TX_GPIO_PORT, COM5_TX_GPIO_PORT}; 
GPIO_TypeDef* COM_RX_PORT[COMn] = {COM1_RX_GPIO_PORT, COM2_RX_GPIO_PORT, COM3_RX_GPIO_PORT, COM4_RX_GPIO_PORT, COM5_RX_GPIO_PORT}; 
uc32 COM_USART_CLK[COMn] = {COM1_CLK, COM2_CLK, COM3_CLK, COM4_CLK, COM5_CLK};
uc32 COM_TX_PORT_CLK[COMn] = {COM1_TX_GPIO_CLK, COM2_TX_GPIO_CLK, COM3_TX_GPIO_CLK, COM4_TX_GPIO_CLK, COM5_TX_GPIO_CLK}; 
uc32 COM_RX_PORT_CLK[COMn] = {COM1_RX_GPIO_CLK, COM2_RX_GPIO_CLK, COM3_RX_GPIO_CLK, COM4_RX_GPIO_CLK, COM5_RX_GPIO_CLK};
uc16 COM_TX_PIN[COMn] = {COM1_TX_PIN, COM2_TX_PIN, COM3_TX_PIN, COM4_TX_PIN, COM5_TX_PIN};
uc16 COM_RX_PIN[COMn] = {COM1_RX_PIN, COM2_RX_PIN, COM3_RX_PIN, COM4_RX_PIN, COM5_RX_PIN};
uc16 COM_IRQ[COMn] = {USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn, UART5_IRQn};

COM_TypeDef printf_COMx;

/**
  * @brief  Inintialization of USART
  * @param  COM: which USART to inialialize
  * @param  br: Baudrate used for USART
  * @retval None
  */
void uart_init(COM_TypeDef COM, u32 br)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM] | RCC_APB2Periph_AFIO, ENABLE);

	if (COM == COM1)
	{
		RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
	}

	else if (COM == COM3)
	{
	/* Enable the USART3 Pins Software Remapping */
		GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);
		RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
	}
	else
	{
		RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
	}

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
	GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

	/* USART configuration */
	USART_InitStructure.USART_BaudRate = br;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(COM_USART[COM], &USART_InitStructure);
	USART_Cmd(COM_USART[COM], ENABLE);
}

/**
  * @brief  Enable the interrupt of USART
  * @param  COM: which USART to enable interrupt
  * @retval None
  */
void uart_interrupt(COM_TypeDef COM)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	#ifdef VECT_TAB_RAM
	NVIC_SetVectorTable(NVIC_VectTab_RAM,0x0);
	#else
	NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0);
	#endif

	NVIC_InitStructure.NVIC_IRQChannel = COM_IRQ[COM];
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Enables the USART receive interrupt */
	USART_ITConfig(COM_USART[COM],USART_IT_RXNE,ENABLE);
}

/**
  * @brief  Enable the function of sending data of Printf via USART
  * @param  COM: which USART to be used for Printf
  * @retval None
  */
void uart_printf_enable(COM_TypeDef COM)
{
	printf_COMx = COM;
}

/**
  * @brief  Disable the function of sending data of Printf via UART
  * @param  None
  * @retval None
  */
void uart_printf_disable(void)
{
	printf_COMx = COM_NULL;
}

/**
  * @brief  Sending one byte of data via USART
  * @param  COM: which USART to be used for sending data
  * @param  data: one byte data to be sent
  * @retval None
  */
void uart_tx_byte(COM_TypeDef COM, uc8 data)
{
	while (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_TC) == RESET); 
	USART_SendData(COM_USART[COM],data);
}

/**
  * @brief  Sending multiple bytes of data via USART
  * @param  COM: which USART to be used for sending data
  * @param  tx_buf: string to be sent
  * @retval None
  */
void uart_tx(COM_TypeDef COM, uc8 * tx_buf, ...)
{
	va_list arglist;
	u8 buf[40], *fp;
	
	va_start(arglist, tx_buf);
	vsprintf((char*)buf, (const char*)tx_buf, arglist);
	va_end(arglist);
	
	fp = buf;
	while (*fp)
		uart_tx_byte(COM,*fp++);
}

/**
  * @brief  Binding of function Printf
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	if (printf_COMx == COM_NULL)
		return ch;
		
	while (USART_GetFlagStatus(COM_USART[printf_COMx], USART_FLAG_TC) == RESET);
	USART_SendData(COM_USART[printf_COMx], (uint8_t) ch);
	return ch;
}
