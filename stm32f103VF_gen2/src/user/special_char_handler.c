#include "special_char_handler.h"

static char special_char_handler_last_char = 0;
static void(*special_char_functions[UCHAR_MAX + 1])(void) = {0};

static void char_bluetooth_decode(u8 id, u8 length, u8* data)
{
  switch (id) {
    case BLUETOOTH_RECEIVE_CHAR_ID:
      if (length == 1) {
				special_char_handler_last_char = data[0];
				if (special_char_functions[(unsigned char)data[0]] != 0){
					special_char_functions[(unsigned char)data[0]]();
				}
      }
    break;
  }
}

char special_char_handler_bt_get_last_char(void)
{
	return special_char_handler_last_char;
}

void special_char_handler_init(void)
{
	  bluetooth_rx_add_filter(BLUETOOTH_RECEIVE_CHAR_ID, 0xFF, char_bluetooth_decode);
}

void register_special_char_function(char c, void(*function)(void))
{
	special_char_functions[(unsigned char)c] = function;
}
