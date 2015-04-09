#include "bluetooth_xbc_mb.h"

static u32 xbc_digital = 0;
static s16 xbc_joy[XBC_JOY_COUNT] = {0};
static u32 last_bt_connection = 0;


static void xbc_bluetooth_decode(u8 id, u8 length, u8* data)
{
    switch (id) {
      
      case BLUETOOTH_XBC_ID: 
          if (length == 8) {
            xbc_digital = data[0] + (data[1] << 8);
            xbc_joy[XBC_JOY_LT] = data[2];
            xbc_joy[XBC_JOY_RT] = data[3];
            xbc_joy[XBC_JOY_LX] = data[4] + (data[5] << 8); 
            xbc_joy[XBC_JOY_LY] = data[6] + (data[7] << 8);
          }
          last_bt_connection = get_full_ticks();
      break;
      
      case BLUETOOTH_XBC_ID + 1:
          if (length == 4) {
            xbc_joy[XBC_JOY_RX] = data[0] + (data[1] << 8); 
            xbc_joy[XBC_JOY_RY] = data[2] + (data[3] << 8);
          }
          last_bt_connection = get_full_ticks();   
      break;
      
    }
}

void bluetooth_xbc_mb_init(void)
{
    bluetooth_rx_add_filter(BLUETOOTH_XBC_ID, 0xF0, xbc_bluetooth_decode);
    last_bt_connection = 0;
    
}

u8 bluetooth_xbc_get_connection(void)
{
    return get_full_ticks() - last_bt_connection <= CAN_XBC_CONNECTION_TIMEOUT_MS;
}


u32 bluetooth_xbc_get_digital(void)
{
  if (!bluetooth_xbc_get_connection()) {
    return 0;
  }
  return xbc_digital;
}


s16 bluetooth_xbc_get_joy_raw(XBC_JOY j)
{
  if (!bluetooth_xbc_get_connection()) {
    return 0;
  }
  return xbc_joy[j];
}

s16 bluetooth_xbc_get_joy(XBC_JOY j)
{
  if (!bluetooth_xbc_get_connection()) {
    return 0;
  }  
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
