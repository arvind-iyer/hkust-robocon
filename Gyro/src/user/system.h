#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "stm32f10x.h"
#include <stdio.h>
#include "ticks.h"
#include "encoder.h"
#include "algorithm.h"
#include "approx_math.h"
#include "gyro.h"
#include "uart.h"
#include "debug.h"

void encoder_init(void);
void set_encoder_zero_all(void);
s32 read_encoder(char number);
void set_encoder_zero(char number);
void read_encoders(int32_t* data_buf);

#endif		/* __SYSTEM_H */
