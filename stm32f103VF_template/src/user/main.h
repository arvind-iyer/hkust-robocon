#ifndef __MAIN_H
#define __MAIN_H

#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_crc.h"

/*** Essential ***/
#include "ticks.h"
#include "battery.h"
#include "delay.h"

#include "buzzer.h"
#include "led.h"
//#include "tft_160x128.h"
#include "lcd_red.h"
#include "system.h"
#include "xbc_mb.h"
#include "button.h"
#include "encoder.h"

/*** Optional ***/
#include "can.h"
#include "can_protocol.h"
#include "uart.h"
#include "approx_math.h"
#include "motor_pwm.h"
#include "servo.h"
#include "gyro.h"
#include "bluetooth.h"





extern u16 ticks_img;
extern u16 seconds_img;

#endif /* __MAIN_H */
