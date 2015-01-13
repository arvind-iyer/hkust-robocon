#ifndef	__BLUETOOTH_H
#define	__BLUETOOTH_H

#include "uart.h"

#define	BLUETOOTH_COM										COM2			/* UART Port */
#define	BLUETOOTH_COM_BR								115200		/* Baudrate */
#define	BLUETOOTH_COM_IRQHandler				void USART2_IRQHandler(void)


/*** BLUETOOTH PROTOCOL ***/
#define	BLUETOOTH_DATA_LENGTH			8		// 0 - 8
#define	BLUETOOTH_PACKAGE_LENGTH	BLUETOOTH_DATA_LENGTH + 5 // + wakeup + data_length + checkbyte*2 + sleep
#define	BLUETOOTH_WAKEUP					0x12
#define	BLUETOOTH_SLEEP						0x34



typedef struct {
	u8 data0;
	u8 mask;
	void (*handler)(u8 length, u8* data);
} BLUETOOTH_RX_FILTER;


void bluetooth_init(void);
void bluetooth_tx_byte(uc8 byte);
void bluetooth_tx(uc8* tx_buf, ...);


void bluetooth_rx_add_filter(u8 data0, u8 mask, void (*handler)(u8 length, u8* data)) ;
u8 bluetooth_rx_state(void);

#define	BLUETOOTH_RX_FILTER_NUM		28

#endif	/* __BLUETOOTH_H */
