#ifndef	__BLUETOOTH_H
#define	__BLUETOOTH_H

#include "usart.h"
#include "ticks.h"
#include "crc.h"
#include "led.h"
#include "gpio.h"
#include "buzzer_song.h"

#define	BLUETOOTH_COM										COM2			/* UART Port */
#define	BLUETOOTH_COM_BR								115200		/* Baudrate */
#define	BLUETOOTH_COM_IRQHandler				void USART2_IRQHandler(void)


/*** BLUETOOTH PROTOCOL ***/
#define	BLUETOOTH_PACKAGE_PRE_LENGTH			3		// wakeup + ID + data_length
#define	BLUETOOTH_PACKAGE_DATA_LENGTH			8		// 0 - 8
#define	BLUETOOTH_PACKAGE_POST_LENGTH			4		// ID + checkbyte * 2 + sleep
#define	BLUETOOTH_PACKAGE_LENGTH					(BLUETOOTH_PACKAGE_PRE_LENGTH+BLUETOOTH_PACKAGE_DATA_LENGTH+BLUETOOTH_PACKAGE_POST_LENGTH)
#define	BLUETOOTH_WAKEUP									0x12
#define	BLUETOOTH_SLEEP										0x34

#define BLUETOOTH_RX_RESET_TIMEOUT				50
#define BLUETOOTH_RX_CHECKBYTES_FLAG			1		// True if rx uses check bytes for verification

#define	BLUETOOTH_STATE_PIN								((const GPIO*) &PB1)
#define	BLUETOOTH_STATE_ON_TICKS					500
typedef struct {
	u8 id;
	u8 mask;
	void (*handler)(u8 id, u8 length, u8* data);
} BLUETOOTH_RX_FILTER;

/*** General ****/
void bluetooth_init(void);
void bluetooth_enable(void);
void bluetooth_disable(void);
u8 bluetooth_get_enabled(void);
	
/*** TX ***/
void bluetooth_tx_byte(uc8 byte);
void bluetooth_tx(const char* tx_buf, ...);
void bluetooth_tx_package(u8 id, u8 data_length, u8* data);

/*** RX ***/
void bluetooth_rx_add_filter(u8 id, u8 mask, void (*handler)(u8 id, u8 length, u8* data));
u8 bluetooth_rx_state(void);
u16 bluetooth_get_data_count(void);

u8 bluetooth_recent_rx_id(void);
u8 bluetooth_recent_rx_data_length(void);
const u8* bluetooth_recent_rx_data(void);

void bluetooth_update(void);
void bluetooth_rx_handler(u8 rx_data);


#define	BLUETOOTH_RX_FILTER_NUM		28

#endif	/* __BLUETOOTH_H */
