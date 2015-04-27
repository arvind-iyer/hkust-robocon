#ifndef __AUTO_TIMER_H
#define __AUTO_TIMER_H
#include "stm32f10x.h"

typedef struct {
	void (*start)(u8 mode);
	void (*update)(void);
} auto_timer;

// robot timer structure
extern auto_timer robot_timer;

#endif
