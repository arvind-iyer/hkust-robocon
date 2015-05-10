#include "send_debug_data.h"
#include "bluetooth.h"
#include "string.h"

void send_string(char* string_to_send)
{
	bluetooth_tx_package(BLUETOOTH_SEND_DEBUG_STRING_ID, strlen(string_to_send), (u8*)string_to_send);
}

void send_string_s(char* string_to_send, int length)
{
	bluetooth_tx_package(BLUETOOTH_SEND_DEBUG_STRING_ID, length, (u8*)string_to_send);
}
