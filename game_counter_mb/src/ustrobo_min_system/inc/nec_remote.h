#ifndef __NEC_REMOTE_H
#define __NEC_REMOTE_H

#include "stm32f10x.h"
#include "nec.h"

#define NEC_REMOTE_NULL_COMMAND   0xFF

typedef struct {
  NEC_Data_TypeDef  \
    /** Essential functions for TV **/
    power,
    mute,
    volume_up,
    volume_down,
    channel_up,
    channel_down,
    last_channel,
    input,
  
    /** Digits **/
    digit0,
    digit1,
    digit2,
    digit3,
    digit4,
    digit5,
    digit6,
    digit7,
    digit8,
    digit9,
  
    /** Arrows **/
    up,
    down,
    left,
    right,
    ok,
    
    /** Menu functions **/
    menu,
    home,
    back,
    exit,
    
    /** Color buttons **/
    red,
    green,
    yellow,
    blue
    ;
} NEC_REMOTE_COMMAND_LIST;

typedef struct {
  u16 address;
  NEC_REMOTE_COMMAND_LIST commands;
} NEC_REMOTE;

extern const NEC_REMOTE NEC_REMOTE_LG;

#endif  /** __NEC_REMOTE_H **/
