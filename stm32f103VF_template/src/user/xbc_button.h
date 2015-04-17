#ifndef	__XBC_BUTTON
#define	__XBC_BUTTON

#include "stm32f10x.h"

u32 xbc_get_received_nonzero_speed_timer(void);
void xbc_button_handler(void);
u32 xbc_get_received_nonzero_angle_timer(void);
#endif
