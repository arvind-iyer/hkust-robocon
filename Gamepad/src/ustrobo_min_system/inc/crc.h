#ifndef	__CRC_H
#define	__CRC_H
#include "stm32f10x.h"

u8 crc16(u8* buffer, const u8* message, int msg_l);

#endif /* __CRC_H */
