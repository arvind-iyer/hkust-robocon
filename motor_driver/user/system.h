#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "system_stm32f10x.h"
#include "misc.h"
// User library
#include "ticks.h"
#include "uart.h"
#include "velocity.h"
#include "motor.h"
#include "led.h"
#include "encoder.h"
#include "can.h"
#include "can_protocol.h"
#include "can_motor.h"

void system_init(u16 sysFreq);
void system_pwm_enable(void);
void system_para_init(void);

#define ADC1_DR_Address    ((u32)0x4001244C)

#endif
