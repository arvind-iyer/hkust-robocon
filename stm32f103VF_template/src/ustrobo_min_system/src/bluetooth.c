/**
  ******************************************************************************
  * @file    bluetooth.c
  * @author  Kenneth Au
  * @version V1.0.0
  * @date    22-January-2015
  * @brief   This file provides Bluetooth (acts as an USART port) functions, 
	*					 including initialization, package transmission with encoding,
	*					 received data package handler with package decoding.
	*
  ******************************************************************************
  * @attention
  *
  * This source is designed for application use. Unless necessary, try NOT to
	* modify the function definition. The constants which are more likely to 
	* vary among different schematics have been placed as pre-defined constant
	* (i.e., "#define") in the header file.
	*
  ******************************************************************************
  */

#include "bluetooth.h"

static u8 bluetooth_enable_flag = 0;
static u8 rx_state = 0;	// 0 as initial state
static u8 rx_data_length = 0;
static u8 rx_data_id = 0;		// The data ID
static u8 rx_check_byte[2] = {0};
static u8 rx_successful_data[BLUETOOTH_PACKAGE_DATA_LENGTH] = {0};
static u16 rx_successful_rx_data_count = 0;
static u32 rx_last_update = 0;
static u32 rx_successful_toggle = 0;

/** Recent successful data **/
static u8 rx_recent_data_id = 0;
static u8 rx_recent_data_length = 0;
static u8 rx_recent_data[8] = {0};

static BLUETOOTH_RX_FILTER rx_filter[BLUETOOTH_RX_FILTER_NUM] = {0};
static u8 rx_filter_count = 0;

USART_TypeDef* BLUETOOTH_USART	= 0;

/**
	* @brief Initialize the Bluetooth USART COM Port with interrupt
	* @param None.
	* @retval None.
	*/
void bluetooth_init(void)
{
	uart_init(BLUETOOTH_COM, BLUETOOTH_COM_BR);
	uart_rx_init(BLUETOOTH_COM,bluetooth_rx_handler);
	rx_filter_count = 0;
	rx_successful_rx_data_count = 0;
	bluetooth_enable_flag = 1;
}

/**
	* @brief Enable Bluetooth transmission
	* @param None
	* @retval None
	*/
void bluetooth_enable(void)
{
	bluetooth_enable_flag = 1;
}

/**
	* @brief Disable Bluetooth transmission
	* @param None
	* @retval None
	*/
void bluetooth_disable(void)
{
	bluetooth_enable_flag = 0;
}

/**
	* @brief Get the current status of Bluetooth transmission (whether it is enabled)
	* @param None
	* @retval The status of Bluetooth (1: enabled, 0: disabled)
	*/
u8 bluetooth_get_enabled(void)
{
	return bluetooth_enable_flag;
}

void bluetooth_tx_byte(uc8 byte)
{
	if (bluetooth_enable_flag) {
		uart_tx_byte(BLUETOOTH_COM, byte);
	}
}

void bluetooth_tx(const char* tx_buf, ...)
{
	va_list arglist;
	u8 buf[40], *fp;
	
	va_start(arglist, tx_buf);
	vsprintf((char*)buf, (const char*)tx_buf, arglist);
	va_end(arglist);
	
	fp = buf;
	while (*fp) {
		uart_tx_byte(BLUETOOTH_COM,*fp++);
  }
}

/**
	* @brief Transmit a package of data through Bluetooth protocol
	* @param id: Package ID
	* @param data_length: 0 - 8
	* @param data: The array[data_length] of data 
	* @retval None
	*/
void bluetooth_tx_package(u8 id, u8 data_length, u8* data)
{
	if (bluetooth_enable_flag) {
		u8 tx_package[BLUETOOTH_PACKAGE_LENGTH];
		u8 tx_state = 0, i;
		if (data_length <= BLUETOOTH_PACKAGE_DATA_LENGTH) {
			tx_package[tx_state++] = BLUETOOTH_WAKEUP;
			tx_package[tx_state++] = id;
			tx_package[tx_state++] = data_length;
			for (i = 0; i < data_length; ++i) {
				tx_package[tx_state++] = data[i];
			}
			tx_package[tx_state++] = id;
			crc16(&tx_package[tx_state], data, data_length);
			tx_state += 2;
			tx_package[tx_state++] = BLUETOOTH_SLEEP;
			
			for (i = 0; i < tx_state; ++i) {
				bluetooth_tx_byte(tx_package[i]);
			}
		}
	}
}

/**
	* @brief Reset the rx state of the bluetooth protocol receival
	* @param None
	* @retval None
	*/
void bluetooth_rx_data_reset(void)
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

void bluetooth_rx_add_filter(u8 id, u8 mask, void (*handler)(u8 id, u8 length, u8* data))
{
	assert_param(rx_filter_count < BLUETOOTH_RX_FILTER_NUM);
	rx_filter[rx_filter_count].id = id;
	rx_filter[rx_filter_count].mask = mask;
	rx_filter[rx_filter_count].handler = handler;
	++rx_filter_count;
}

/**
	* @brief RX_STATE getter (mainly for debug)
	*/
u8 bluetooth_rx_state(void)
{
	return rx_state;
}

/**
	* @brief Get the number of successful received Bluetooth package
	* @param None
	* @retval The number of successful received Bluetooth package 
	*/
u16 bluetooth_get_data_count(void)
{
	return rx_successful_rx_data_count;
}

void bluetooth_rx_handler(u8 rx_data)
{
		switch (rx_state) {
			case 0: 
				// STATE [0]: Wakeup command
				if (rx_data == BLUETOOTH_WAKEUP) {
					++rx_state;
				} else {
					bluetooth_rx_data_reset();
				}
			break;
				
			case 1:
				// STATE [1]: ID
				rx_data_id = rx_data;
				++rx_state;
			break;
			
			case 2:
				// STATE [2]: Data length
				if (rx_data <= BLUETOOTH_PACKAGE_DATA_LENGTH) {
					// Correct data length (0 - 8)
					++rx_state;
					rx_data_length = rx_data;
				} else {
					bluetooth_rx_data_reset();
				}
			break;
			

			default:
				if (rx_state < BLUETOOTH_PACKAGE_LENGTH) {
					if (rx_state < BLUETOOTH_PACKAGE_PRE_LENGTH + rx_data_length) {	// STATE [3..n-4] : Getting the data array
						// Data array
						rx_successful_data[rx_state-BLUETOOTH_PACKAGE_PRE_LENGTH] = rx_data;
						++rx_state;
					} else if (rx_state == BLUETOOTH_PACKAGE_PRE_LENGTH + rx_data_length) {	// STATE [n-3]: ID (for checking)
						if (rx_data_id == rx_data) {
							// Correct ID
							++rx_state;
						} else {
							bluetooth_rx_data_reset();
						}

					} else if (rx_state == BLUETOOTH_PACKAGE_PRE_LENGTH + rx_data_length + 1) {	// STATE [n-2]: Check byte 1
						if (BLUETOOTH_RX_CHECKBYTES_FLAG) {
							crc16(rx_check_byte, rx_successful_data, rx_data_length);
							if (rx_data == rx_check_byte[0]) {
								// Correct check byte 1
								++rx_state;
							} else {
								bluetooth_rx_data_reset();
							}
						} else {
							++rx_state;
						}
						
					} else if (rx_state == BLUETOOTH_PACKAGE_PRE_LENGTH + rx_data_length + 2) {	// STATE [n-1]: Check byte 2
						if (BLUETOOTH_RX_CHECKBYTES_FLAG) {
							if (rx_data == rx_check_byte[1]) {
								// Correct check byte 1
								++rx_state;
							} else {
								bluetooth_rx_data_reset();
							}
						} else {
							++rx_state;
						}
					} else if (rx_state == BLUETOOTH_PACKAGE_PRE_LENGTH + rx_data_length + 3) {	// STATE [n]: Sleep command
						if (rx_data == BLUETOOTH_SLEEP) {
							// Successful data package transmission
							void bluetooth_data_handler(u8 id, u8 length, u8* data);
							void bluetooth_rx_successful(void);
							// Copy it to the recent data sets
							rx_recent_data_id = rx_data_id;
							rx_recent_data_length = rx_data_length;
							for (u8 i = 0; i < 8; ++i) {
								rx_recent_data[i] = rx_successful_data[i];
							}
							bluetooth_data_handler(rx_data_id, rx_data_length, rx_successful_data);
							bluetooth_rx_successful();
							bluetooth_rx_data_reset();
						} else {
							bluetooth_rx_data_reset();
						}
					} else {
						bluetooth_rx_data_reset();
					}
				} else {
					bluetooth_rx_data_reset();
				}
			break;
		}
	rx_last_update = get_seconds() * 1000 + get_ticks();	
}


/**
	* @brief Function handler for bluetooth RX 
	* @param id: Data package ID
	* @param length: Data length
	* @param data: Data array
	* @retval None.
	*/
void bluetooth_data_handler(u8 id, u8 length, u8* data)
{
	u8 i = 0;
	for (i = 0; i < rx_filter_count; ++i) {
		// Check whether the filtered data0 fits any data[0]
		if ((rx_filter[i].id & rx_filter[i].mask) == (id & rx_filter[i].mask)) {
			rx_filter[i].handler(id, length, data);
			return;
		}
	}
}

/**
	* @brief Call this when a package receive is successful. 
	*	@param None.
	* @retval None.
	*/
void bluetooth_rx_successful(void)
{
	++rx_successful_rx_data_count;
	rx_successful_toggle = !rx_successful_toggle;
	led_control(LED_D1, (LED_STATE) rx_successful_toggle);
}

u8 bluetooth_recent_rx_id(void)
{
	return rx_recent_data_id;
}

u8 bluetooth_recent_rx_data_length(void)
{
	return rx_recent_data_length;
}

const u8* bluetooth_recent_rx_data(void)
{
	return rx_recent_data;
}

/**
	* @brief Regular check of the bluetooth rx. 
	* 				Reset the rx_state if there is not any data received after BLUETOOTH_RX_RESET_TIMEOUT ms.
	*					Also toggle the LED for every successful receive.
	* @param None.
	* @retval None.
	*/
void bluetooth_update(void)
{
	u32 current_time = get_seconds() * 1000 + get_ticks();
	if (current_time - rx_last_update > BLUETOOTH_RX_RESET_TIMEOUT) {
		bluetooth_rx_data_reset();
		if (rx_successful_toggle == LED_ON) {
			led_control(LED_D1, LED_OFF);
		}
	}
	
}
