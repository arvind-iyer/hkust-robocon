#include "uart.h"

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

USART_TypeDef* printf_UARTx;

USART_TypeDef* COM_USART[COMn] = {USART1, USART2, USART3, UART4, UART5}; 

GPIO_TypeDef* COM_TX_PORT[COMn] = {COM1_TX_GPIO_PORT, COM2_TX_GPIO_PORT, COM3_TX_GPIO_PORT, COM4_TX_GPIO_PORT, COM5_TX_GPIO_PORT};
 
GPIO_TypeDef* COM_RX_PORT[COMn] = {COM1_RX_GPIO_PORT, COM2_RX_GPIO_PORT, COM3_RX_GPIO_PORT, COM4_RX_GPIO_PORT, COM5_RX_GPIO_PORT};
 
uc32 COM_USART_CLK[COMn] = {COM1_CLK, COM2_CLK, COM3_CLK, COM4_CLK, COM5_CLK};

uc32 COM_TX_PORT_CLK[COMn] = {COM1_TX_GPIO_CLK, COM2_TX_GPIO_CLK, COM3_TX_GPIO_CLK, COM4_TX_GPIO_CLK, COM5_TX_GPIO_CLK};
 
uc32 COM_RX_PORT_CLK[COMn] = {COM1_RX_GPIO_CLK, COM2_RX_GPIO_CLK, COM3_RX_GPIO_CLK, COM4_RX_GPIO_CLK, COM5_RX_GPIO_CLK};

uc16 COM_TX_PIN[COMn] = {COM1_TX_PIN, COM2_TX_PIN, COM3_TX_PIN, COM4_TX_PIN, COM5_TX_PIN};

uc16 COM_RX_PIN[COMn] = {COM1_RX_PIN, COM2_RX_PIN, COM3_RX_PIN, COM4_RX_PIN, COM5_RX_PIN};

uc16 COM_IRQ[COMn] = {USART1_IRQn, USART2_IRQn, USART3_IRQn, 0, 0};

u8 uart_printf_enabled = 0;

void uart_init(COM_TypeDef COM)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM] | RCC_APB2Periph_AFIO, ENABLE);

	if (COM == COM1)
	{
		RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
	}
	else if (COM == COM3)
	{
	/* Enable the USART3 Pins Software Remapping */
		//GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);
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
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(COM_USART[COM], &USART_InitStructure);
	USART_Cmd(COM_USART[COM], ENABLE);
}

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

void printf_switch_uart(USART_TypeDef* UARTx)
{
	printf_UARTx = UARTx;
	uart_printf_enabled = 1;
}


void uart_tx_byte(USART_TypeDef* UARTx, uc8 data)
{
	while (USART_GetFlagStatus(UARTx, USART_FLAG_TC) == RESET); 
	USART_SendData(UARTx,data);
}

void uart_tx(USART_TypeDef* UARTx, uc8 * tx_buf, ...)
{
	va_list arglist;
	u8 buf[40], *fp;
	
	va_start(arglist, tx_buf);
	vsprintf((char*)buf, (const char*)tx_buf, arglist);
	va_end(arglist);
	
	fp = buf;
	while (*fp)
		uart_tx_byte(UARTx,*fp++);
}

PUTCHAR_PROTOTYPE
{
	if (!uart_printf_enabled)
		return ch;

	while (USART_GetFlagStatus(printf_UARTx, USART_FLAG_TC) == RESET);
	USART_SendData(printf_UARTx, (uint8_t) ch);
	return ch;
}

void uart_printf_enable(void) {
	uart_printf_enabled = 1;
}

void uart_printf_disable(void) {
	uart_printf_enabled = 0;
}
