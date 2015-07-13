#ifndef __MAIN_H
#define __MAIN_H

#include <stdio.h>
#include <stdlib.h>
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
#include "xbc_mb.h"
#include "button.h"
#include "encoder.h"
#include "system_test.h"

/*** Optional ***/
#include "can_protocol.h"
#include "usart.h"
#include "approx_math.h"
#include "servo.h"
#include "gyro.h"
#include "bluetooth.h"
#include "robocon.h"
#include "wheel_base.h"
#include "ultrasonic.h"
#include "game_counter.h"
#include "timer.h"
#include "uart_com.h"
#include "rtc.h"

#define TIMER_MODE_COUNT    4

typedef enum {
  PREPARATION_TIME,   /** 1 min **/
  SERVING_TIME,       /** 5 second **/
  SETTING_TIME,       /** 15 seconds **/
  TIMEOUT_TIME        /** 30 seconds **/
} GAME_MODE;

static const u8 game_mode_time[] = {
  60, 5, 15, 30
};


#endif /* __MAIN_H */
