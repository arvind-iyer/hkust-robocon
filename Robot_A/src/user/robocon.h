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
//#define	ANGLE_LOCK_LED_GPIO		((GPIO*) &PE3)
#define	AUTO_SERVE_LED_GPIO		((GPIO*) &PC12)

#define	GYRO_LED_GPIO					((GPIO*) &PE9)
#define	UPPER_RACKET_GPIO			((GPIO*) &PE10)

#define	FLASH_SHUTTLE_DROP_DELAY_OFFSET			0
#define	FLASH_SERVING_HIT_SPEED_OFFSET			1 
#define	FLASH_WHEEL_BASE_SPEED_MODE_OFFSET	2
#define	FLASH_ABS_ANGLE_MODE_OFFSET					3
#define	FLASH_ANGLE_LOCK_MODE_OFFSET				4
#define	FLASH_SHUTTLE_DROP_DELAY_2_OFFSET			5
#define	FLASH_SERVING_HIT_SPEED_2_OFFSET			6
#define	FLASH_SERVING_SET_OFFSET							7

#define	AUTO_SERVE_PRE_DELAY								7000		// 7000 ms
void robocon_init(void);
void robocon_main(void);

#endif	/* __ROBOCON_H */
