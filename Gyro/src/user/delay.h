#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

void _delay_us( u32 nus);
void _delay_ms( u16 nms );

void simple_delay1_ms( void );
void simple_delay10_us( void  );

#endif /* __DELAY_H */
