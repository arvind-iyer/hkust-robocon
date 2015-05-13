#include "send_debug_data.h"
#include "bluetooth.h"
#include "string.h"

#define NON_DATA_BYTES 7

void send_string(char* string_to_send)
{
	int package_length = strlen(string_to_send) + NON_DATA_BYTES;
	u8 tx_package[package_length];
	u8 tx_state = 0, i;
		
	tx_package[tx_state++] = BLUETOOTH_WAKEUP;
	tx_package[tx_state++] = BLUETOOTH_SEND_DEBUG_STRING_ID;
	tx_package[tx_state++] = strlen(string_to_send);
	for (i = 0; i < strlen(string_to_send); ++i) {
		tx_package[tx_state++] = string_to_send[i];
	}
	tx_package[tx_state++] = BLUETOOTH_SEND_DEBUG_STRING_ID;
	crc16(&tx_package[tx_state], (u8*)string_to_send, strlen(string_to_send));
	tx_state += 2;
	tx_package[tx_state++] = BLUETOOTH_SLEEP;
		
	for (i = 0; i < tx_state; ++i) {
		bluetooth_tx_byte(tx_package[i]);
	}
}

void send_string_s(char* string_to_send, int length)
{
	int package_length = length + NON_DATA_BYTES;
	u8 tx_package[package_length];
	u8 tx_state = 0, i;
		
	tx_package[tx_state++] = BLUETOOTH_WAKEUP;
	tx_package[tx_state++] = BLUETOOTH_SEND_DEBUG_STRING_ID;
	tx_package[tx_state++] = length;
	for (i = 0; i < length; ++i) {
		tx_package[tx_state++] = string_to_send[i];
	}
	tx_package[tx_state++] = BLUETOOTH_SEND_DEBUG_STRING_ID;
	crc16(&tx_package[tx_state], (u8*)string_to_send, length);
	tx_state += 2;
	tx_package[tx_state++] = BLUETOOTH_SLEEP;
		
	for (i = 0; i < tx_state; ++i) {
		bluetooth_tx_byte(tx_package[i]);
	}
}
