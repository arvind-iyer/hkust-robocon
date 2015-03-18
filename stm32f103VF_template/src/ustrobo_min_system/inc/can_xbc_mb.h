#ifndef __CAN_XBC_H
#define __CAN_XBC_H

#include "stm32f10x.h"
#include "can_protocol.h"
#include "ticks.h"

#define CAN_XBC_BASE        0x090

typedef enum {
  XBC_DISCONNECTED, 
  XBC_CAN_CONNECTED,
  XBC_ALL_CONNECTED
} XBC_CONNECTION_MODE;

#define XBC_JOY_COUNT           6
#define XBC_JOY_DEADZONE_MIN    ((s16) 4000)
#define XBC_JOY_DEADZONE_MAX    ((s16) 30000)
#define XBC_JOY_SCALE           1000  // Self-defined scale
typedef enum {
  XBC_JOY_LT,         // 1-byte
  XBC_JOY_RT,         // 1-byte
  XBC_JOY_LX,         // 2-byte
  XBC_JOY_LY,         // 2-byte
  XBC_JOY_RX,         // 2-byte
  XBC_JOY_RY          // 2-byte
} XBC_JOY;


void can_xbc_mb_init(void);
XBC_CONNECTION_MODE xbc_get_connection(void);
u32 xbc_get_digital(void);

s16 xbc_get_joy_raw(XBC_JOY j);
s16 xbc_get_joy(XBC_JOY j);

#endif  /* __CAN_XBC_H */
