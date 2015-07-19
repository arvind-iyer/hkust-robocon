#ifndef	__FLASH_H
#define	__FLASH_H

#include "stm32f4xx_flash.h"
#include "stm32f4xx.h"


void writeFlash(u8 address_num,u16 data);
u16 readFlash();


#endif /* __FLASH_H */