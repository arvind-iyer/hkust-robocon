#ifndef	__SYSTEM_TEST_H
#define	__SYSTEM_TEST_H

#include <stdio.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_crc.h"

/*** Essential ***/
#include "ticks.h"
#include "battery.h"
#include "delay.h"

#include "buzzer.h"
#include "led.h"
#include "tft.h"
#include "interface.h"
#include "can_xbc_mb.h"
#include "button.h"
#include "encoder.h"

/*** Optional ***/
#include "can_protocol.h"
#include "can_motor.h"
#include "usart.h"
#include "approx_math.h"
#include "servo.h"
#include "gyro.h"
#include "bluetooth.h"
#include "robocon.h"
#include "wheel_base.h"
#include "ultrasonic.h"
#include "bluetooth_xbc_mb.h"
#include "xbc_mb.h"


void bluetooth_test(void);
void ascii_test(void);
void motor_test(void);
void battery_test(void);
void position_test(void);
void button_test(void);
void buzzer_test(void);
void can_test(void);
void can_xbc_test(void);
void xbc_test(void);
void bluetooth_xbc_test(void);
void gpio_pin_test(void);
void uart_test(void);
void ultra_test(void);
void snake(void);
 
#endif /* __SYSTEM_TEST_H */
