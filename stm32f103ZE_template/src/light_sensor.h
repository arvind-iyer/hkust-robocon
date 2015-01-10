#ifndef __LIGHT_SENSOR_H
#define __LIGHT_SENSOR_H

#include "uart.h"
#include "ticks.h"
#include "delay.h"
#include "debug.h"
#include "approx_math.h"

#define LS_UART  COM3
#define X -1
#define LS_TL 0x01 
#define LS_TR 0x02
#define LS_BL 0x04
#define LS_BR 0x08

extern u8 ls_test;

void ls_init(void);
u8 get_ls(void);
u8 test_ls(s8 top_left, s8 top_right, s8 bottom_left, s8 bottom_right);

#endif		/* __LIGHT_SENSOR_H */
