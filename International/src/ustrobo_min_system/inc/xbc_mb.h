#ifndef __XBC_MB_H
#define __XBC_MB_H

#include "stm32f10x.h"

#define XBC_JOY_COUNT           6
#define XBC_JOY_DEADZONE_MIN    ((s16) 6000)
#define XBC_JOY_DEADZONE_MAX    ((s16) 32000)
#define XBC_JOY_SCALE           1000  // Self-defined scale

typedef enum {
  XBC_JOY_LT,         // 1-byte
  XBC_JOY_RT,         // 1-byte
  XBC_JOY_LX,         // 2-byte
  XBC_JOY_LY,         // 2-byte
  XBC_JOY_RX,         // 2-byte
  XBC_JOY_RY          // 2-byte
} XBC_JOY;

/* 
 * xbox controller digital signal (used with xbc_digital)
 * 16 bit form 	
 */
#define XBC_UP		  0x0001
#define XBC_DOWN	  0x0002
#define XBC_LEFT	  0x0004
#define XBC_RIGHT	  0x0008
#define XBC_START	  0x0010
#define XBC_BACK	  0x0020
#define XBC_L_JOY	  0x0080
#define XBC_R_JOY	  0x0040
#define XBC_LB		  0x0100
#define XBC_RB	  	0x0200
#define XBC_XBOX	  0x0400
#define XBC_A		    0x1000
#define XBC_B		    0x2000
#define XBC_X		    0x4000
#define XBC_Y		    0x8000

typedef enum {
  XBC_DISCONNECTED,
  XBC_CAN_CONNECTED,
  XBC_BLUETOOTH_CONNECTED
} XBC_CONNECTION;

typedef enum {
  XBC_CAN_FIRST,
  XBC_BLUETOOTH_FIRST
} XBC_PRIORITY;

void xbc_mb_init(XBC_PRIORITY priority);
XBC_CONNECTION xbc_get_connection(void);
XBC_PRIORITY xbc_get_priority(void);
u32 xbc_get_digital(void);
s16 xbc_get_joy_raw(XBC_JOY j);
s16 xbc_get_joy(XBC_JOY j);

#endif  /** __XBC_MB_H **/
