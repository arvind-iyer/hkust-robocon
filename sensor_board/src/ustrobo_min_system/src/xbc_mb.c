#include "xbc_mb.h"
#include "bluetooth_xbc_mb.h"
#include "can_xbc_mb.h"

static XBC_PRIORITY xbc_priority = XBC_BLUETOOTH_FIRST;

/**
  * @brief Initialize xbox contollers (both of Bluetooth && CAN)
  * @param priority: If two controllers are connected, which controller is used
  * @retval None.
  */
void xbc_mb_init(XBC_PRIORITY priority)
{
  xbc_priority = priority;
  bluetooth_xbc_mb_init();
  can_xbc_mb_init();
  can_xbc_mb_tx_enable(true); 
}


/**
  * @brief Get xbc connection state
  * @retval Connection state. @ref XBC_CONNECTION
  */
XBC_CONNECTION xbc_get_connection(void)
{
  switch (xbc_priority) 
  {
    case XBC_BLUETOOTH_FIRST:
      if (bluetooth_xbc_get_connection()) {
        return XBC_BLUETOOTH_CONNECTED;
      } else if (can_xbc_get_connection() == CAN_XBC_ALL_CONNECTED) {
        return XBC_CAN_CONNECTED;
      }
    break;
      
    case XBC_CAN_FIRST:
      if (can_xbc_get_connection() == CAN_XBC_ALL_CONNECTED) {
        return XBC_CAN_CONNECTED;
      } else if (bluetooth_xbc_get_connection()) {
        return XBC_BLUETOOTH_CONNECTED;
      }
    break;
  }
  return XBC_DISCONNECTED;
}

/**
  * @brief Get the current XBC priority.
  * @retval The current XBC priority. @ref XBC_PRIORITY.
  */
XBC_PRIORITY xbc_get_priority(void)
{
  return xbc_priority;
}

/**
  * @brief Get XBC digital. (Used with bitwise operators)
  * @retval xbc_digital (bitwise).
  */
u32 xbc_get_digital(void)
{
  switch (xbc_get_connection()) {
    case XBC_CAN_CONNECTED:
      return can_xbc_get_digital();

    
    case XBC_BLUETOOTH_CONNECTED:
      return bluetooth_xbc_get_digital();

    default:
      return 0;
  }

}


/**
  * @brief Get XBC joystick value (with deadzone correction).
  * @param j: joystick ID. @ref XBC_JOY
  * @retval The joystick value. (0 - 255 for 1-byte joystick, -XBC_JOY_SCALE to XBC_JOY_SCALE for 2-byte joystick)
  */
s16 xbc_get_joy(XBC_JOY j)
{
  switch (xbc_get_connection()) {
    case XBC_CAN_CONNECTED:
      return can_xbc_get_joy(j);

    
    case XBC_BLUETOOTH_CONNECTED:
      return bluetooth_xbc_get_joy(j);

    default:
      return 0;
  }
}


/**
  * @brief Get the raw XBC joystick value (NO deadzone correction).
  * @param j: joystick ID. @ref XBC_JOY
  * @retval The joystick value. (0 - 255 for 1-byte joystick, -32768 to 32 for 2-byte joystick)
  */
s16 xbc_get_joy_raw(XBC_JOY j)
{
  switch (xbc_get_connection()) {
    case XBC_CAN_CONNECTED:
      return can_xbc_get_joy_raw(j);

    
    case XBC_BLUETOOTH_CONNECTED:
      return bluetooth_xbc_get_joy_raw(j);

    default:
      return 0;
  }
}

