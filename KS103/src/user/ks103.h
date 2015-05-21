#ifndef	__KS103_H 
#define	__KS103_H 

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "usart.h"
#include <stdbool.h>
#define	KS103_COUNT										2
#define	KS103_BR											9600

#define	KS103_TTL_ADDRESS_DEFAULT			0xe8
#define	KS103_TTL_REGISTER_DEFAULT		0x02


typedef enum {
	KS103_NULL,
	KS103_SEND_TTL_ADDRESS,
	KS103_SEND_TTL_ADDRESS_DELAY,
	KS103_SEND_REGISTER,
	KS103_SEND_REGISTER_DELAY,
	KS103_SEND_DETECTING_COMMAND,
	KS103_RECEIVE_HIGH_BIT,
	KS103_RECEIVE_LOW_BIT
} KS103_STATE;

typedef struct {
	const COM_TypeDef COMx;
	u8 range_command;
	u8 raw_data_low, raw_data_high;
	u16 range;
	u16 last_sample_second;
	u16 sample_count_tmp;
	u16 sample_count;
	
	KS103_STATE state;
	void (*rx_handler) (u8 data);
} KS103_TypeDef;

void ks103_init(void);
bool ks103_tx(u8 id);
void ks103_tx_force(u8 id); 

const KS103_TypeDef* ks103_get(u8 id);

#endif	/* __KS103_H */
