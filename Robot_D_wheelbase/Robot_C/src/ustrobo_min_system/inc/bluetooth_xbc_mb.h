#ifndef __BLUETOOTH_XBC_MB_H
#define __BLUETOOTH_XBC_MB_H

#include "bluetooth.h"
#include "can_xbc_mb.h"
#include "xbc_mb.h"

#define BLUETOOTH_XBC_ID                        0x90

#define BLUETOOTH_XBC_CONNECTION_TIMEOUT_MS     300

void bluetooth_xbc_mb_init(void);
u8 bluetooth_xbc_get_connection(void);
u32 bluetooth_xbc_get_digital(void);
s16 bluetooth_xbc_get_joy_raw(XBC_JOY j);
s16 bluetooth_xbc_get_joy(XBC_JOY j);

#endif  /** __BLUETOOTH_XBC_MB_H **/
