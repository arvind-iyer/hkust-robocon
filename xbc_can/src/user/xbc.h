#ifndef __XBC_H
#define __XBC_H

#include "stm32f10x.h" 
#include "usb_api.h"

#define XBC_DATA_RAW_COUNT    20
#define XBC_DATA_COUNT        13

#define CAN_XBC_BASE   0x90

u8 get_xbc_data(u8 i);
void xbc_tx_data(void);
void xbc_loop(void);


#endif  /** __XBC_H **/
