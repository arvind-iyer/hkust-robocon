#ifndef	__ROBOCON_H
#define	__ROBOCON_H

#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_crc.h"

/*** Essential ***/
#include "ticks.h"
#include "delay.h"

#include "buzzer.h"
#include "led.h"
#include "tft.h"
#include "interface.h"
#include "xbc_mb.h"
#include "button.h"
#include "encoder.h"
  //'C' or 'D'
#define ROBOT 'C'







/*** Optional ***/
#include "can_protocol.h"
#include "usart.h"
#include "approx_math.h"
#include "servo.h"
#include "gyro.h"
#include "bluetooth.h"
#include "wheel_base.h"
#include "racket.h"
#include "xbc_mb.h"
#include "log.h"



void robocon_main(void);
//bool get_xbc_input_allowed(void);
//void set_xbc_input_allowed(bool);
#endif	/* __ROBOCON_H */
