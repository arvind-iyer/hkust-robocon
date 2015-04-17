#include "nec_remote.h"

const NEC_REMOTE NEC_REMOTE_NULL = {
  NEC_REMOTE_NULL_COMMAND,
  {
    /** Essential functions for TV **/
    .power        = NEC_REMOTE_NULL_COMMAND,
    .mute         = NEC_REMOTE_NULL_COMMAND,
    .volume_up    = NEC_REMOTE_NULL_COMMAND,
    .volume_down  = NEC_REMOTE_NULL_COMMAND,
    .channel_up   = NEC_REMOTE_NULL_COMMAND,
    .channel_down = NEC_REMOTE_NULL_COMMAND,
    .last_channel = NEC_REMOTE_NULL_COMMAND,
    .input        = NEC_REMOTE_NULL_COMMAND,
  
    /** Digits **/
    .digit0       = NEC_REMOTE_NULL_COMMAND,
    .digit1       = NEC_REMOTE_NULL_COMMAND,
    .digit2       = NEC_REMOTE_NULL_COMMAND,
    .digit3       = NEC_REMOTE_NULL_COMMAND,
    .digit4       = NEC_REMOTE_NULL_COMMAND,
    .digit5       = NEC_REMOTE_NULL_COMMAND,
    .digit6       = NEC_REMOTE_NULL_COMMAND,
    .digit7       = NEC_REMOTE_NULL_COMMAND,
    .digit8       = NEC_REMOTE_NULL_COMMAND,
    .digit9       = NEC_REMOTE_NULL_COMMAND,
  
    /** Arrows **/
    .up           = NEC_REMOTE_NULL_COMMAND,
    .down         = NEC_REMOTE_NULL_COMMAND,
    .left         = NEC_REMOTE_NULL_COMMAND,
    .right        = NEC_REMOTE_NULL_COMMAND,
    .ok           = NEC_REMOTE_NULL_COMMAND,
    
    /** Menu functions **/
    .menu         = NEC_REMOTE_NULL_COMMAND,
    .home         = NEC_REMOTE_NULL_COMMAND,
    .back         = NEC_REMOTE_NULL_COMMAND,
    .exit         = NEC_REMOTE_NULL_COMMAND,
    
    /** Color buttons **/
    .red          = NEC_REMOTE_NULL_COMMAND,
    .green        = NEC_REMOTE_NULL_COMMAND,
    .yellow       = NEC_REMOTE_NULL_COMMAND,
    .blue         = NEC_REMOTE_NULL_COMMAND
  }
};

const NEC_REMOTE NEC_REMOTE_NEC = {
  NEC_REMOTE_NULL_COMMAND,
  {
    /** Essential functions for TV **/
    .power        = NEC_REMOTE_NULL_COMMAND,
    .mute         = NEC_REMOTE_NULL_COMMAND,
    .volume_up    = 0x15,
    .volume_down  = 0x07,
    .channel_up   = 0x47,
    .channel_down = 0x45,
    .last_channel = 0x46,
    .input        = NEC_REMOTE_NULL_COMMAND,
  
    /** Digits **/
    .digit0       = 0x16,
    .digit1       = 0x0C,
    .digit2       = 0x18,
    .digit3       = 0x5E,
    .digit4       = 0x08,
    .digit5       = 0x1C,
    .digit6       = 0x5A,
    .digit7       = 0x42,
    .digit8       = 0x52,
    .digit9       = 0x4A,
  
    /** Arrows **/
    .up           = NEC_REMOTE_NULL_COMMAND,
    .down         = NEC_REMOTE_NULL_COMMAND,
    .left         = NEC_REMOTE_NULL_COMMAND,
    .right        = NEC_REMOTE_NULL_COMMAND,
    .ok           = NEC_REMOTE_NULL_COMMAND,
    
    /** Menu functions **/
    .menu         = NEC_REMOTE_NULL_COMMAND,
    .home         = NEC_REMOTE_NULL_COMMAND,
    .back         = NEC_REMOTE_NULL_COMMAND,
    .exit         = NEC_REMOTE_NULL_COMMAND,
    
    /** Color buttons **/
    .red          = NEC_REMOTE_NULL_COMMAND,
    .green        = NEC_REMOTE_NULL_COMMAND,
    .yellow       = NEC_REMOTE_NULL_COMMAND,
    .blue         = NEC_REMOTE_NULL_COMMAND
  }
};


const NEC_REMOTE NEC_REMOTE_LG = {
  0x04,
  {
    /** Essential functions for TV **/
    .power        = 0x08,
    .mute         = 0x09,
    .volume_up    = 0x02,
    .volume_down  = 0x03,
    .channel_up   = 0x00,
    .channel_down = 0x01,
    .last_channel = 0x1A,
    .input        = 0x0B,
		
    /** Digits **/
    .digit0       = 0x10,
    .digit1       = 0x11,
    .digit2       = 0x12,
    .digit3       = 0x13,
    .digit4       = 0x14,
    .digit5       = 0x15,
    .digit6       = 0x16,
    .digit7       = 0x17,
    .digit8       = 0x18,
    .digit9       = 0x19,
  
    /** Arrows **/
    .up           = 0x40,
    .down         = 0x41,
    .left         = 0x07,
    .right        = 0x06,
    .ok           = 0x44,
    
    /** Menu functions **/
    .menu         = 0x43,
    .home         = 0x7C,
    .back         = 0x28,
    .exit         = 0x5B,
    
    /** Color buttons **/
    .red          = 0x72,
    .green        = 0x71,
    .yellow       = 0x63,
    .blue         = 0x61
  }
};

