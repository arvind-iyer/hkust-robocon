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
#include "ultrasonic_mb.h"
  //'C' or 'D'
#define ROBOT 'D'


/*** Optional ***/
#include "can_protocol.h"
#include "usart.h"
#include "approx_math.h"
#include "servo.h"
#include "gyro.h"
#include "bluetooth.h"
#include "wheel_base.h"
#include "sensors.h"
#include "racket.h"
#include "serve.h"
#include "xbc_mb.h"
#include "log.h"
#include "nec_mb.h"
#include "send_debug_data.h"
#include "flash.h"
#include "ultrasonic.h" 

#define TR_ACC_FAC			ROBOT=='C'?50:54
#define TL_ACC_FAC			ROBOT=='C'?50:63 			
#define BR_ACC_FAC			ROBOT=='C'?50:27 
#define BL_ACC_FAC			ROBOT=='C'?50:27 

#define	E_STOP_BUTTON		(&PE3)
static void robot_cd_common_function(void);
bool is_force_terminate(void);
void robocon_main(void);
static void robot_c_function_controls(void);
static void robot_d_function_controls(void);
bool robot_xbc_controls(void);
//bool get_xbc_input_allowed(void);
//void set_xbc_input_allowed(bool);
#endif	/* __ROBOCON_H */
