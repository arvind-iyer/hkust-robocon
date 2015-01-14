#ifndef	__BLUETOOTH_H
#define	__BLUETOOTH_H

#include "uart.h"
#include "crc.h"

#define	BLUETOOTH_COM										COM2			/* UART Port */
#define	BLUETOOTH_COM_BR								115200		/* Baudrate */
#define	BLUETOOTH_COM_IRQHandler				void USART2_IRQHandler(void)


/*** BLUETOOTH PROTOCOL ***/
#define	BLUETOOTH_PACKAGE_PRE_LENGTH			3		// wakeup + ID + data_length
#define	BLUETOOTH_PACKAGE_DATA_LENGTH			8		// 0 - 8
#define	BLUETOOTH_PACKAGE_POST_LENGTH			4		// ID + checkbyte * 2 + sleep
#define	BLUETOOTH_PACKAGE_LENGTH	(BLUETOOTH_PACKAGE_PRE_LENGTH+BLUETOOTH_PACKAGE_DATA_LENGTH+BLUETOOTH_PACKAGE_POST_LENGTH)
#define	BLUETOOTH_WAKEUP					0x01
#define	BLUETOOTH_SLEEP						0x04



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
