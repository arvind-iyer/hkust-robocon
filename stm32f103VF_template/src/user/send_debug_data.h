#ifndef __SEND_DEBUG_DATA
#define __SEND_DEBUG_DATA

#define BLUETOOTH_SEND_DEBUG_STRING_ID 0x99

void send_string(char* string_to_send, int length);

void send_string_s(char* string_to_send, int length);

#endif