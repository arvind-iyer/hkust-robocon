#include "bluetooth.h"

static u8 rx_state = 0;	// 0 as initial state
static u8 rx_data_length = 0;
static u8 rx_data_id = 0;		// The data ID
static u8 rx_check_byte[2] = {0};
static u8 rx_successful_data[BLUETOOTH_PACKAGE_DATA_LENGTH] = {0};

static BLUETOOTH_RX_FILTER rx_filter[BLUETOOTH_RX_FILTER_NUM] = {0};
static u8 rx_filter_count = 0;

USART_TypeDef* BLUETOOTH_USART	= 0;

void bluetooth_init(void)
{
	uart_init(BLUETOOTH_COM, BLUETOOTH_COM_BR);
	uart_interrupt(BLUETOOTH_COM);
	BLUETOOTH_USART = COM_USART[BLUETOOTH_COM];
	rx_filter_count = 0;
}

void bluetooth_tx_byte(uc8 byte)
{
	uart_tx_byte(BLUETOOTH_COM, byte);
}

void bluetooth_tx(uc8* tx_buf, ...)
{
	va_list arglist;
	u8 buf[40], *fp;
	
	va_start(arglist, tx_buf);
	vsprintf((char*)buf, (const char*)tx_buf, arglist);
	va_end(arglist);
	
	fp = buf;
	while (*fp)
		uart_tx_byte(BLUETOOTH_COM,*fp++);
}


void rx_data_reset(void)
{
	u8 i = BLUETOOTH_PACKAGE_DATA_LENGTH;
	rx_state = 0;
	
	while (i--) {
		rx_successful_data[i] = 0;
	}
	
	rx_data_id = 0;
	rx_check_byte[0] = rx_check_byte[1] = 0;
}

/***
	rx_data[0] 			= BLUETOOTH_WAKEUP;
	rx_data[1] 			= ID;
	rx_data[2] 			= data_length;			//	0 - 8
	rx_data[3..n-4] = data[x[n]]	// the data
	rx_data[n-3]		= ID;
	rx_data[n-2] 		= check_byte0;
	rx_data[n-1] 		= check_byte1;
	rx_data[n]  		= BLUETOOTH_SLEEP;
***/

void bluetooth_rx_add_filter(u8 data0, u8 mask, void (*handler)(u8 length, u8* data)) {
	assert_param(rx_filter_count < BLUETOOTH_RX_FILTER_NUM);
	rx_filter[rx_filter_count].data0 = data0;
	rx_filter[rx_filter_count].mask = mask;
	rx_filter[rx_filter_count].handler = handler;
	++rx_filter_count;
}

u8 bluetooth_rx_state(void)
{
	return rx_state;
}

BLUETOOTH_COM_IRQHandler
{
	u8 rx_data;
	if (USART_GetITStatus(BLUETOOTH_USART, USART_IT_RXNE) != RESET) {
		rx_data = (u8)USART_ReceiveData(BLUETOOTH_USART);
		
		switch (rx_state) {
			case 0: 
				// STATE [0]: Wakeup command
				if (rx_data == BLUETOOTH_WAKEUP) {
					++rx_state;
				} else {
					rx_data_reset();
				}
			break;
				
			case 1:
				// STATE [1]: ID
				rx_data_id = rx_data;
			break;
			
			case 2:
				// STATE [2]: Data length
				if (rx_data <= BLUETOOTH_PACKAGE_DATA_LENGTH) {
					// Correct data length (0 - 8)
					++rx_state;
					rx_data_length = rx_data;
				} else {
					rx_data_reset();
				}
			break;
			

			default:
				if (rx_state < BLUETOOTH_PACKAGE_LENGTH) {
					if (rx_state < BLUETOOTH_PACKAGE_PRE_LENGTH + rx_data_length) {	// STATE [3..n-4] : Getting the data array
						// Data array
						rx_successful_data[rx_state-BLUETOOTH_PACKAGE_PRE_LENGTH] = rx_data;
						++rx_state;
					} else if (rx_state == BLUETOOTH_PACKAGE_PRE_LENGTH + rx_data_length + 1) {	// STATE [n-3]: ID (for checking)
						if (rx_data_id == rx_data) {
							// Correct ID
							++rx_state;
						} else {
							rx_data_reset();
						}

					} else if (rx_state == BLUETOOTH_PACKAGE_PRE_LENGTH + rx_data_length + 2) {	// STATE [n-2]: Check byte 1
						crc16(rx_check_byte, &rx_successful_data[rx_state-BLUETOOTH_PACKAGE_PRE_LENGTH], rx_data_length);
						if (rx_data == rx_check_byte[0]) {
							// Correct check byte 1
							++rx_state;
						} else {
							rx_data_reset();
						}
						
					} else if (rx_state == BLUETOOTH_PACKAGE_PRE_LENGTH + rx_data_length + 3) {	// STATE [n-1]: Check byte 2
						if (rx_data == rx_check_byte[1]) {
							// Correct check byte 1
							++rx_state;
						} else {
							rx_data_reset();
						}
					} else if (rx_state == BLUETOOTH_PACKAGE_PRE_LENGTH + rx_data_length + 4) {	// STATE [n]: Sleep command
						if (rx_data == BLUETOOTH_SLEEP) {
							// Successful data package transmission
							void bluetooth_data_handler(u8 length, u8* data);
							bluetooth_data_handler(rx_data_length, rx_successful_data);
							rx_data_reset();
						} else {
							rx_data_reset();
						}
					} else {
						rx_data_reset();
					}
				} else {
					rx_data_reset();
				}
			break;
		}
	}		
}

void bluetooth_data_handler(u8 length, u8* data)
{
	u8 i = 0;
	for (i = 0; i < rx_filter_count; ++i) {
		// Check whether the filtered data0 fits any data[0]
		if ((rx_filter[i].data0 & rx_filter[i].mask) == (data[0] & rx_filter[i].mask)) {
			rx_filter[i].handler(length, data);
			return;
		}
	}
}

