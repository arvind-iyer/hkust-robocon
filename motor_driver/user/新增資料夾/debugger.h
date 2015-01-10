#ifndef DEBUGGER_H
#define DEBUGGER_H

#include <stdio.h>
#include "stm32f10x.h"
#include "sysinit.h"
#include "misc.h"
#include "stm32f10x_rcc.h"

#include "main.h"

void system_shake_hand (u8 shake);
/*
void send_cmd(u8 _byte);
void send_float(float* _num);
void send_stop(void);
void debugger_msg(void);
void set_and_send_float32(char index, float data);
*/
void update_token(void);
void send_debug_msg(u8 index, float data);
void send_float(u8 cmd, float* data);
void send_3_floats(u8 cmd, float* data1, float* data2, float* data3);
void debugger_msg(void);

#define stop_byte1	0xfe
#define stop_byte2	0xff
#define buf_size	200

#endif
