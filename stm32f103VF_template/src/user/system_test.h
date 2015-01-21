#ifndef	__SYSTEM_TEST_H
#define	__SYSTEM_TEST_H

#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_crc.h"

/*** Essential ***/
#include "ticks.h"
#include "battery.h"
#include "delay.h"

#include "buzzer.h"
#include "led.h"
#include "lcd_red.h"
#include "interface.h"
#include "xbc_mb.h"
#include "button.h"
#include "encoder.h"

/*** Optional ***/
#include "can_protocol.h"
#include "uart.h"
#include "approx_math.h"
#include "servo.h"
#include "gyro.h"
#include "bluetooth.h"
#include "robocon.h"
#include "wheel_base.h"

void bluetooth_test(void);
void ascii_test(void);
void battery_test(void);
void position_test(void);

#endif /* __SYSTEM_TEST_H */
