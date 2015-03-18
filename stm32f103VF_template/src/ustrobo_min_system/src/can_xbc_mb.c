#include "can_xbc_mb.h"

static u32 xbc_digital = 0;
static s16 xbc_joy[XBC_JOY_COUNT] = {0};
static u32 last_can_connection = 0;
static XBC_CONNECTION_MODE xbc_connection = XBC_DISCONNECTED;

static void can_xbc_mb_decoding(CanRxMsg msg)
{
    switch (msg.StdId) {
      case CAN_XBC_BASE:
          if (msg.DLC == 8) {
            xbc_digital = msg.Data[0] + (msg.Data[1] << 8);
            xbc_joy[XBC_JOY_LT] = msg.Data[2];
            xbc_joy[XBC_JOY_RT] = msg.Data[3];
            xbc_joy[XBC_JOY_LX] = msg.Data[4] + (msg.Data[5] << 8); 
            xbc_joy[XBC_JOY_LY] = msg.Data[6] + (msg.Data[7] << 8);
            xbc_connection = XBC_ALL_CONNECTED;
          } else {
            xbc_digital = 0;
            xbc_connection = XBC_CAN_CONNECTED;
          }
          last_can_connection = get_full_ticks();
      break;
      
      case CAN_XBC_BASE+1:
        if (msg.DLC == 5) {
          xbc_joy[XBC_JOY_RX] = msg.Data[0] + (msg.Data[1] << 8); 
          xbc_joy[XBC_JOY_RY] = msg.Data[2] + (msg.Data[3] << 8);
          last_can_connection = get_full_ticks();
          xbc_connection = XBC_ALL_CONNECTED;
        }
      break;
    }
}
  
XBC_CONNECTION_MODE xbc_get_connection(void)
{
  return xbc_connection;
}

u32 xbc_get_digital(void)
{
    return xbc_digital;
}

s16 xbc_get_joy_raw(XBC_JOY j) 
{
    return xbc_joy[j];
}

s16 xbc_get_joy(XBC_JOY j)
{
  switch (j) {
    case XBC_JOY_LT:
    case XBC_JOY_RT:
      return xbc_joy[j];

    
    case XBC_JOY_LX:
    case XBC_JOY_LY:
    case XBC_JOY_RX:
    case XBC_JOY_RY:
      if (xbc_joy[j] >= -XBC_JOY_DEADZONE_MIN && xbc_joy[j] <= XBC_JOY_DEADZONE_MIN) {
        return 0;
      } else if (xbc_joy[j] < -XBC_JOY_DEADZONE_MAX) {
        return -XBC_JOY_SCALE;
      } else if (xbc_joy[j] > XBC_JOY_DEADZONE_MAX) {
        return XBC_JOY_SCALE;
      } else {
        if (xbc_joy[j] > 0) {
          return (xbc_joy[j] - XBC_JOY_DEADZONE_MIN) * XBC_JOY_SCALE / (XBC_JOY_DEADZONE_MAX - XBC_JOY_DEADZONE_MIN);
        } else {
          return (xbc_joy[j] - -XBC_JOY_DEADZONE_MIN) * XBC_JOY_SCALE / (XBC_JOY_DEADZONE_MAX - XBC_JOY_DEADZONE_MIN);
        }
      }
      
  }
  return 0;
}

void can_xbc_mb_init(void) 
{
    xbc_connection = XBC_DISCONNECTED;
    xbc_digital = 0;
    can_rx_add_filter(CAN_XBC_BASE, CAN_RX_MASK_DIGIT_0_3, can_xbc_mb_decoding);
}

