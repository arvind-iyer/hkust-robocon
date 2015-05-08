#ifndef __CAN_XBC_H
#define __CAN_XBC_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "can_protocol.h"
#include "ticks.h"
#include "tft.h"
#include "xbc_mb.h"

#define CAN_XBC_BASE                0x090
#define CAN_XBC_MB_TX_LCD_ID        0x100

#define CAN_XBC_CONNECTION_TIMEOUT_MS   200

#define RGB565TORGB323(RGB565)  (((RGB565 >> 8) & 0xE0) | ((RGB565 >> 6) & 0x18) | ((RGB565 >> 2) & 0x07))
#define RGB323TORGB565(RGB323)  (((((RGB323 >> 5) & 0x07) * 0x1F / 0x07) << 11) | ((((RGB323 >> 3) & 0x03) * 0x3F / 0x03) << 5) | (((RGB323) & 0x07) * 0x1F / 0x07))
#define RGB565TOGRAY4(RGB565) (((((RGB565 >> 12)&0x0F)+((RGB565 >> 7)&0x0F)+((RGB565 >> 1)&0x0F))/3) & 0x0F)
#define GRAY4TORGB565(GRAY4) ((GRAY4 << 12) | (GRAY4 << 7) | (GRAY4 << 1))

typedef enum {
  CAN_XBC_DISCONNECTED,   /*!< XBC Board CAN Disconnected ***/
  CAN_XBC_CAN_CONNECTED,  /*!< XBC Board CAN connected, but the XBC USB on the XBC board disconnected ***/
  CAN_XBC_ALL_CONNECTED   /*!< XBC Board All connected ***/
} CAN_XBC_CONNECTION_MODE;



typedef struct {
  u16 color, bg_color;
  char text;
  
} XBC_LCD_DATA;

void can_xbc_mb_init(void);
void can_xbc_mb_tx_enable(bool flag);
CAN_XBC_CONNECTION_MODE can_xbc_get_connection(void);
u32 can_xbc_get_digital(void);

s16 can_xbc_get_joy_raw(XBC_JOY j);
s16 can_xbc_get_joy(XBC_JOY j);
void can_xbc_mb_lcd_tx(void);
#endif  /* __CAN_XBC_H */

