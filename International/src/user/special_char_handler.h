
#ifndef __SPECIAL_CHAR_HANDLER_H
#define __SPECIAL_CHAR_HANDLER_H

#define UCHAR_MAX 255
#define BLUETOOTH_RECEIVE_CHAR_ID           0x70    // RX

#include "stm32f10x.h"
#include "can_motor.h"
#include "bluetooth.h"
#include "ticks.h"

char special_char_handler_bt_get_last_char(void);
void special_char_handler_init(void);
void register_special_char_function(char c, void(*function)(void));

#endif
