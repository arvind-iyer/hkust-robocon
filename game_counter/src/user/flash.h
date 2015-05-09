#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f10x.h"
#include "stm32f10x_flash.h"

void write_flash(u16 offset, s32 data);
s32 read_flash(u16 offset);

#endif  /* __FLASH_H */
