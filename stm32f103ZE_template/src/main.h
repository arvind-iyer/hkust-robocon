#ifndef __MAIN_H
#define __MAIN_H

#include <stdio.h>
#include "stm32f10x.h"

/*** Essential ***/
#include "ticks.h"
#include "battery.h"
#include "delay.h"

#include "debug.h"
#include "lcd_red.h"
#include "system.h"
#include "xbc_mb.h"
#include "button.h"

/*** Optional ***/
#include "uart.h"
#include "approx_math.h"
#include "motor_pwm.h"
#include "servo.h"
#include "lm629.h"
#include "gyro.h"
#include "light_sensor.h"

// robot
#include "drive.h"



extern u16 ticks_img;
extern u16 seconds_img;

#endif /* __MAIN_H */
