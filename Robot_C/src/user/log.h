
#ifndef __LOG_H
#define __LOG_H

#include "stm32f10x.h"
#include "string.h"
#include "tft.h"

#define LOG_SPACE 4;

void log(char* pstr, s32 val);
void log_update();
//void log_update();

#endif
