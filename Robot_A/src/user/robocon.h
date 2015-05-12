#ifndef	__ROBOCON_H
#define	__ROBOCON_H

#include <stdio.h>
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

/*** Optional ***/
#include "can_protocol.h"
#include "usart.h"
#include "approx_math.h"
#include "servo.h"
#include "gyro.h"
#include "bluetooth.h"
#include "wheel_base.h"
#include "wheel_base_pid.h"

#define	SERVING_LED_GPIO			((GPIO*) &PD2)
#define	US_AUTO_LED_GPIO			((GPIO*) &PE5)
#define	ANGLE_LOCK_LED_GPIO		((GPIO*) &PC12)

void robocon_init(void);
void robocon_main(void);

#endif	/* __ROBOCON_H */
