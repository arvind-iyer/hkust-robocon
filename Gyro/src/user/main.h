#ifndef _MAIN_H_
#define _MAIN_H_

#include "stm32f10x.h"
#include <stdio.h>
#include "ticks.h"
#include "debug.h"
#include "delay.h"
#include "encoder.h"
#include "uart.h"
#include "gyro.h"
#include "system.h"
//#include "algorithm.h"

#define GYRO_COM	COM3

/*** Command List ***/
#define GYRO_WAKEUP				0x01

#define GYRO_UPDATE				0x10
#define GYRO_CAL				0x20
#define GYRO_POS_SET			0x30
#define GYRO_AUTO_UPDATE		0x40

/*** Reply Command List ***/
#define GYRO_REPLY				0x50
#define GYRO_UPDATED			0x80

#define GYRO_COMMAND_LENGTH		4

#define GYRO_REPLAY_SET_POS		0x00
#define GYRO_REPLAY_CAL			0x01

void send_pos(void);
void send_reply(u8 data);
void uart_update(void);

#endif /* _MAIN_H_ */
