#ifndef	__FLASH_H
#define	__FLASH_H

#include "stm32f10x.h"

#define	FLASH_PAGE_SIZE_KB				1024	
#define	FLASH_OFFSET_SIZE_KB			4

void write_flash(u16 offset, s32 data);
s32 read_flash(u16 offset);

#endif	/* __FLASH_H */
