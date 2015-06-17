#ifndef	__NEC_MB_H
#define	__NEC_MB_H

#include "can_protocol.h"

#define	NEC_DEVICE_COUNT			8
#define	NEC_CAN_ID						0x220
#define	NEC_CAN_RX_TIMEOUT		300

typedef u8 NEC_Data_TypeDef;

typedef struct {
  NEC_Data_TypeDef address, command;
	u8 state;
} NEC_Msg;

void nec_mb_init(void);
const NEC_Msg* nec_get_msg(u8 id);

#endif	/* __NEC_MB_H */
